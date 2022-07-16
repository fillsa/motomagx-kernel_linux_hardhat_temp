/*
 * Copyright 2004 Freescale Semiconductor, Inc.
 * Copyright 2006-2007 Motorola, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * Date     Author    Comment
 * 10/2006  Motorola  Initial version.  Combined code from mxcfb.c and 
 *                    mxcfb_epson.c for the SDC and ADC IPU interfaces,
 *                    and then added support for HVGA smart displays.
 * 11/2006  Motorola  Updated DSM modes and added ESD recovery fix.
 * 11/2006  Motorola  Fix for Serializer lockup when entering and exiting DSM
 * 12/2006  Motorola  Cleaned up SYSFS interface functions
 * 01/2007  Motorola  Added support for dynamic IPU memory pool size
 * 01/2007  Motorola  Move pixel-clk & serializer control to hvga_disable_channel
 *                    and hvga_reconfigure_channel. Also make sure switch_panel_state
 *                    is called even when there's a DMA's state update is needed
 * 02/2007  Motorola  Commonized code to add Sharp support
 * 03/2007  Motorola  Added global function mxcfb_set_static_state for video drivers
 * 03/2007  Motorola  Fix setting of yres_virtual for overlay device for double-buffering only
 * 03/2007  Motorola  Updated framework due to new panel command spreadsheets
 * 04/2007  Motorola  Added FBIOROTATEIMAGE ioctl
 * 04/2007  Motorola  Removed Sharp hack to reinitialize panel on bootup
 * 04/2007  Motorola  Updated DSM strategy to use FBIOPARTIALMODE instead of suspend/resume
 * 05/2007  Motorola  Removed the hack to reinitialize Sharp on bootup (again)
 * 05/2007  Motorola  Moved call to ipu_disable_channel;
 * 05/2007  Motorola  Added mxcfb_fbmem_blank to fix audio drop out
 * 06/2007  Motorola  Fixed issue with ESD routine occuring outside of vsync
 * 06/2007  Motorola  Lido display performance optimizations
 * 06/2007  Motorola  Added kthread to process flip and key events quickly
 * 06/2007  Motorola  Added new ioctl to store the user-backlight preference
 * 06/2007  Motorola  Set global alpha to 255 before panel state transition if overlay is in use
 * 06/2007  Motorola  Use sem to sync up panel state transition  and plane blending at flip switching
 * 06/2007  Motorola  Updated ESD recovery routine to eliminate light pulse on Sharp
 */

/*!
 * @file mxcfb_hvga.c
 *
 * @brief MXC Frame buffer driver for HVGA (2x240x320) 
 *
 * @ingroup Framebuffer
 */
 
/*!
 * Include files
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/lights_backlight.h> /* Phone's backlights ioctls */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/kthread.h>
#include <linux/reboot.h>
#include <linux/mpm.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/mot-gpio.h>

#include "mxcfb_hvga.h"

#if !defined(CONFIG_MOT_FEAT_GPIO_API_LCD) || \
    !defined(CONFIG_MOT_FEAT_GPIO_API_SERIALIZER)
#error Must define CONFIG_MOT_FEAT_GPIO_API_LCD and CONFIG_MOT_FEAT_GPIO_API_SERIALIZER
#endif

#include <asm/mot-gpio.h>

#if defined(CONFIG_MOT_FEAT_IPU_IOCTL_EZX_COMPAT)
#include <linux/console.h>	/* acquire_console_sem() */
#endif

#include <asm/arch/mxc_pm.h>

/*
 * Debug Macros
 */
#define MXCFB_NAME      "MXCFB_HVGA"
/* #define MXCFB_DEBUG */

#ifdef MXCFB_DEBUG

#define DDPRINTK(fmt, args...) printk(KERN_ERR"%s :: %d :: %s - " fmt, \
                                __FILE__,__LINE__,__FUNCTION__ , ## args)
#define DPRINTK(fmt, args...) printk("[%d:%s] %s: " fmt, current->pid, current->comm, __FUNCTION__ , ## args)

#define FUNC_START	DPRINTK(" func start\n")
#define FUNC_END	DPRINTK(" func end\n")

#define FUNC_ERR        printk(KERN_ERR"%s :: %d :: %s  err= %d \n", __FILE__, \
                        __LINE__,__FUNCTION__ ,err)

#else				//MXCFB_DEBUG

#define DDPRINTK(fmt, args...)  do {} while(0)
#define DPRINTK(fmt, args...)   do {} while(0)

#define FUNC_START
#define FUNC_END

#endif				//MXCFB_DEBUG

struct mxcfb_data {
        struct fb_info *fbi;
        struct fb_info *fbi_ovl;
        struct fb_info *fbi_cli;
        struct fb_info *fbi_iram;
        volatile int32_t vsync_flag;
        volatile int32_t sys1_eof_flag;
#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
	volatile int32_t esd_flag;
#endif
        wait_queue_head_t vsync_wq;
        wait_queue_head_t sys1_eof_wq;
        bool suspended;
        int backlight_level;
};

static uint32_t def_vram = 0;
static struct mxcfb_data mxcfb_drv_data;
static struct panel_specific_functions * panel;
struct global_state mxcfb_global_state; 

/*
 * mxcfb_panel is a pointer to either hvga_panel or qvga_panel. When
 *     the flip is closed, mxcfb_panel will point to qvga_panel to
 *     initialize the display with QVGA dimensions.  When the full HVGA
 *     display is in use, mxcfb_panel is set to hvga_panel.
 */
static const struct panel_info * mxcfb_panel;

static struct task_struct *flipkeyd_task;
static int notifier_flip_state;  /* Last flip status from flipkey notifier. */
static int notifier_key_pressed; /* True if a key has recently been pressed. */

/* Forward declarations */
static int mxcfb_blank(int blank, struct fb_info *fbi);
#if defined(CONFIG_FB_MXC_OVERLAY)
static int mxcfb_ovl_open(struct fb_info *fbi, int user);
#endif
static irqreturn_t mxcfb_irq_handler(int irq, void *dev_id, struct pt_regs *regs);
static int mxcfb_exit_low_power(struct partial_mode_info * pm_info);



/*
 * Set up the panel information pointers for the specific panel hardware being used.
 * These functions should be defined in hardware-specific hvga_{panel}.c files.
 */
static void setup_panel_pointers(void)
{
	uint32_t reg;

	reg = reg_read(0x0);
	DPRINTK("MXCFB panel device code = 0x%X\n", reg);

	/* Default to E02 timings */
	if (reg == 0x38C2) {
		panel = &sharp_panel_functions;
	} else {
		panel = &E02_panel_functions;
	}
	mxcfb_panel = panel->hvga_panel_info;
}

/*********************************************************/

static uint32_t bpp_to_pixfmt(int bpp)
{
	uint32_t pixfmt = 0;
	switch (bpp) {
	case 24:
#if defined(CONFIG_MOT_FEAT_IPU_BGRA6666)
		pixfmt = IPU_PIX_FMT_BGRA6666;
#elif defined(CONFIG_MOT_FEAT_FB_MXC_RGB)
		pixfmt = IPU_PIX_FMT_RGB24;
#else
		pixfmt = IPU_PIX_FMT_BGR24;
#endif
		break;
	case 32:
#if defined(CONFIG_MOT_FEAT_FB_MXC_RGB)
		pixfmt = IPU_PIX_FMT_RGB32;
#else
		pixfmt = IPU_PIX_FMT_BGR32;
#endif
		break;
	case 16:
#if defined(CONFIG_MOT_FEAT_FB_MXC_RGB) 
		/* The V4L2 pixel packing definition of BGR565 is reversed */
		pixfmt = IPU_PIX_FMT_BGR565;
#else
		pixfmt = IPU_PIX_FMT_RGB565;
#endif
		break;
	}
	return pixfmt;
}

static void mxcfb_init_sys1_channel(void)
{
        ipu_channel_params_t params;

        /* Needed to turn on ADC clock for panel init */
        memset(&params, 0, sizeof(params));
        params.adc_sys1.disp = mxcfb_get_disp_num();
        params.adc_sys1.ch_mode = WriteTemplateNonSeq;
        params.adc_sys1.out_left = panel->qvga_panel_info->left_offset; 
        params.adc_sys1.out_top = panel->qvga_panel_info->top_offset;
        ipu_init_channel(ADC_SYS1, &params);
}

/*!
 * Function to initialize the serial display interface and send
 * all the necessary initialization commands to the display.
 */
static void 
mxcfb_init_serial_display(void)
{
        int msb;
        int panel_stride;
	uint32_t disp_num = mxcfb_get_disp_num();

        /* Init DI interface */
        msb = fls(mxcfb_panel->width);
        if (!(mxcfb_panel->width & ((1UL << msb) - 1)))
                msb--;       // Already aligned to power 2
        panel_stride = 1UL << msb;
        ipu_adc_init_panel(disp_num,
                           mxcfb_panel->width + mxcfb_panel->left_offset, 
                           mxcfb_panel->height + mxcfb_panel->middle_porch_lines +
			   mxcfb_panel->top_offset, panel->disp_conf->pix_fmt, 
			   panel_stride, panel->disp_conf->sig, panel->disp_conf->addressing_mode, 
			   panel->disp_conf->vsync_width, panel->disp_conf->vsync_mode);

        ipu_adc_init_ifc_timing(disp_num, true, 
                           panel->disp_conf->read_cycle_time, panel->disp_conf->read_up_time,
			   panel->disp_conf->read_down_time, panel->disp_conf->read_latch_time,
			   panel->disp_conf->read_pixel_clock);
        ipu_adc_init_ifc_timing(disp_num, false, 
                           panel->disp_conf->write_cycle_time, panel->disp_conf->write_up_time,
			   panel->disp_conf->write_down_time, 0, 0);

	mxcfb_init_sys1_channel();

        FUNC_END;
}

static irqreturn_t mxcfb_sys1_eof_irq_handler(int irq, void *dev_id,
		struct pt_regs *regs)
{
	ipu_disable_irq(IPU_IRQ_ADC_SYS1_EOF);

	if (ipu_adc_set_update_mode(ADC_SYS1, IPU_ADC_REFRESH_NONE,
				0, 0, 0) < 0) {
		printk("MXCFB Error disabling auto refresh.\n");
	}

	mxcfb_drv_data.sys1_eof_flag = 1;
        wake_up_interruptible(&mxcfb_drv_data.sys1_eof_wq);

	return IRQ_HANDLED;
}

/* 
 * Send one frame of data via the ADC SPI interface. This is used during DSM 
 * (partial mode) where we send data through the SPI interface instead of RGB.
 */
int mxcfb_update_spi_frame(/*bool wait_for_eof*/void)
{
	ssize_t memsize = mxcfb_drv_data.fbi_cli->fix.smem_len;
	int end_y;
	int retval = 0;

	/* If we are not in partial mode, send the whole frame */
	end_y = mxcfb_global_state.partial_coords.end_y;
	if (end_y == 0) {
		end_y = panel->qvga_panel_info->height - 1;
	}

	ipu_init_channel_buffer(ADC_SYS1, IPU_INPUT_BUFFER,
			bpp_to_pixfmt(mxcfb_drv_data.fbi_cli->var.bits_per_pixel),
			panel->qvga_panel_info->width,
			end_y - mxcfb_global_state.partial_coords.start_y + 1,
			panel->qvga_panel_info->width,
			IPU_ROTATE_NONE,
			(void*)mxcfb_drv_data.fbi_cli->fix.smem_start,
			NULL);

	ipu_select_buffer(ADC_SYS1, IPU_INPUT_BUFFER, 0);
	ipu_enable_channel(ADC_SYS1);

	if (ipu_adc_set_update_mode(ADC_SYS1, IPU_ADC_AUTO_REFRESH_SNOOP, 30,
				mxcfb_drv_data.fbi->fix.smem_start, &memsize) < 0) {
		printk("mxcfb: Error enabling auto refesh.\n");
	}

/*	if (wait_for_eof) {*/
	mxcfb_drv_data.sys1_eof_flag = 0;
	ipu_clear_irq(IPU_IRQ_ADC_SYS1_EOF);
	ipu_enable_irq(IPU_IRQ_ADC_SYS1_EOF);
	/* Wait up to 333*/
	retval = wait_event_interruptible_timeout(mxcfb_drv_data.sys1_eof_wq, 
			(mxcfb_drv_data.sys1_eof_flag != 0), 333 * HZ / 1000);
	if (retval == 0 && !mxcfb_drv_data.sys1_eof_flag) {
		printk("IPU_IRQ_ADC_SYS1_EOF: timeout\n");
		retval = -ETIME;
	} else {
		retval = 0;
/*
	} else {
		if (ipu_adc_set_update_mode(ADC_SYS1, IPU_ADC_REFRESH_NONE,
					0, 0, 0) < 0) {
			printk("MXCFB Error disabling auto refresh.\n");
		}
	}
*/	
	}

	return retval;
}

#ifdef CONFIG_MOT_FEAT_POWERUP_LOGO
extern u32 mot_mbm_is_ipu_initialized;
extern u32 mot_mbm_ipu_buffer_address;
#endif

/* 
 * Returns the refresh mode mask for the passed in panel state.
 */
refresh_mode_t panel_to_refresh_mask(panel_t panel) 
{
	refresh_mode_t retval;
	switch (panel) {
		case PANEL_OFF:
			retval = REFRESH_OFF;
			break;
		case MAIN_PANEL:
			retval = MAIN_FULL_REFRESH;
			break;
		case CLI_PANEL:
			retval = CLI_FULL_REFRESH;
			break;
		case (MAIN_PANEL | CLI_PANEL):
			retval = MAIN_FULL_REFRESH | CLI_FULL_REFRESH;
			break;
		default: 
			retval = -EINVAL;
	}
	return retval;
}

/*
 * Returns the current refresh mode that is actually being DMA'd to the panel
 */
refresh_mode_t get_legal_refresh_mode(void) 
{
	BUG_ON(!(mxcfb_global_state.panel_state & (MAIN_PANEL | CLI_PANEL)) && 
			mxcfb_global_state.panel_state != PANEL_OFF);

	return (mxcfb_global_state.refresh_mode & 
			panel_to_refresh_mask(mxcfb_global_state.panel_state));
}

/*
 * Returns HVGA_MODE if both panels are enabled, or QVGA_MODE if not.  
 * The return value is not effected by DMA refresh; only by panel state.
 */
uint32_t panel_state_to_hvga_mode(void) 
{
	uint32_t retval;

	if (mxcfb_global_state.panel_state == (MAIN_PANEL | CLI_PANEL)) {
		retval = HVGA_MODE;
	} else { 
		retval = QVGA_MODE;
	}
	return retval;
}

/*
 * Returns HVGA_MODE if both framebuffers are actively being DMA'd to 
 * the panel, or QVGA_MODE if not.
 */
uint32_t refresh_mode_to_hvga_mode(void) 
{
	uint32_t retval;
	refresh_mode_t refresh;

	refresh = get_legal_refresh_mode();
	if (refresh == (MAIN_FULL_REFRESH | CLI_FULL_REFRESH)) {
		retval = HVGA_MODE;
	} else {
		retval = QVGA_MODE;
	}
	return retval;
}

/*!
 * Helper function to send initialization commands to the display
 *
 * @param       commands     Table of init commands to send
 */
void display_init(const struct panel_init_commands * commands)
{
	int i = 0;
	int disp = mxcfb_get_disp_num();

	while (commands[i].mode != INVALID) {
		switch (commands[i].mode) {
		    case CMD:
			    ipu_adc_write_cmd(disp, CMD, commands[i].index, &(commands[i].data), 1);
			    break;
		    case DAT:
			    ipu_adc_write_cmd(disp, DAT, commands[i].index, &(commands[i].data), 1);
			    break;
		    case CMD_ONLY:
			    ipu_adc_write_cmd(disp, CMD, commands[i].index, NULL, 0);
			    break;
		    default:
			    BUG_ON(!panel->custom_command);
			    panel->custom_command(disp, &(commands[i]));
		}
		if (commands[i].msleep != 0) {
			msleep(commands[i].msleep);
		}
		i++;
	}
}

/* 
 * Write a panel register via the ADC SPI Low-Level Access interface 
 */
void reg_write(uint32_t index, uint32_t command)
{
	ipu_adc_write_cmd(mxcfb_get_disp_num(), CMD, index, &(command), 1);
}

/*
 * Read a panel register via the ADC SPI Low-Level Access interface 
 */
uint32_t reg_read(uint32_t index)
{
	return ipu_adc_read_cmd(mxcfb_get_disp_num(), index);
}

/* 
 * Continually read a panel register via the ADC SPI Low-Level Access 
 * interface until the value masked with 'mask' is equal to 'compare'.  
 * Maximum timeout of 50ms as determined by the worst case panel read
 * situation during Sharp DSM (mode 6).
 */
int poll_read(uint32_t index, uint32_t mask, uint32_t compare) 
{
	int i;

	/* 20us was chosen through cold hard experimentation */
	udelay(20);
	/* Maximum delay is 50ms in mode 6 with Sharp tango read */
	for (i=0; i < 50; i++) {
		if ((mask & reg_read(index)) == compare) {
			return 0;
		}
		msleep(1);
	}
	return -EIO;
}



#if defined(CONFIG_FB_MXC_OVERLAY)
/*
 * Updates the SDC overlay window to either the MAIN or CLI
 * panel, depending on the current state of overlay_panel.
 *
 * If panel is not either MAIN_PANEL or CLI_PANEL, BUG will be
 *     called.
 *
 * Preconditions: Requires the mxcfb_global_state.g_sem lock to be held.
 */
static void set_overlay_window(void)
{
        struct mxcfb_info * mxc_fbi = mxcfb_drv_data.fbi_ovl->par;
        if( (mxcfb_global_state.overlay_panel != PANEL_OFF) &&
                        (mxc_fbi->open_count != 0) ){
                ipu_enable_channel(mxc_fbi->ipu_ch);
        }
}
#endif /* defined(CONFIG_FB_MXC_OVERLAY) */


/*
 * Open the main framebuffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @param       user    Set if opened by user or clear if opened by kernel
 */
static int
mxcfb_open(struct fb_info *fbi, int user)
{
        int retval = 0;
	struct mxcfb_info * mxc_fbi_cur = (struct mxcfb_info *)fbi->par;
        struct mxcfb_info * mxc_fbi = (struct mxcfb_info *)mxcfb_drv_data.fbi->par;
        struct mxcfb_info * mxc_fbi_cli = (struct mxcfb_info *)mxcfb_drv_data.fbi_cli->par;

#ifdef CONFIG_MOT_FEAT_POWERUP_LOGO
	char __iomem* io_remapped_logo;
	__u32 smem_len;
#endif

        FUNC_START;

        if(fbi == mxcfb_drv_data.fbi_iram) {
                mxc_fbi_cur->ipu_ch_irq = IPU_IRQ_SDC_BG_EOF;
                mxc_fbi_cur->ipu_ch = MEM_SDC_BG;
                mxc_fbi_cur->cur_ipu_buf = 0;
                DPRINTK("Opening fbi_iram: Success. Return\n");
                return 0;
        }

#if defined(CONFIG_FB_MXC_OVERLAY)
        if(fbi == mxcfb_drv_data.fbi_ovl) {
                return mxcfb_ovl_open(fbi, user);
	}
#endif

        if ( (mxc_fbi->open_count == 0) && (mxc_fbi_cli->open_count == 0) )
        {
#ifdef CONFIG_MOT_FEAT_POWERUP_LOGO
                if (mot_mbm_is_ipu_initialized) {
                        /*
                         * Check if mot_mbm_ipu_buffer_address != this-framebuffer's smem_start 
                         * NOTE: fbi->fix.smem_start may not be equal to MXCIPU_MEM_ADDRESS depending on whether
                         * IRAM is available on the hw
                         * */
                        if( mot_mbm_ipu_buffer_address && 
                                mot_mbm_ipu_buffer_address != (u32)fbi->fix.smem_start) {
                                
				/* smem_len should account for both main and CLI framebuffers */
				smem_len = mxcfb_drv_data.fbi->fix.smem_len + mxcfb_drv_data.fbi_cli->fix.smem_len;

                                DPRINTK("MXCIPU_MEM_ADDRESS = 0x%08lX\n", MXCIPU_MEM_ADDRESS);
                                DPRINTK("smem_len= %d\n", smem_len);
                                
                                /* Copy the logo only if:
                                 *      a) MXCIPU_MEM_ADDRESS <= mot_mbm_ipu_buffer_address
                                 *      b) MXCIPU_MEM_ADDRESS + frame_size <= MXCIPU_MEM_ADDRESS + MXCIPU_MEM_SIZE
                                 *      c) dest-address (fb0->fix.smem_start) <= source (mot_mbm_ipu_buffer_address)
                                 *              --because memcpy_fromio() doesnot do reverse copy
                                 * */
                                if(MXCIPU_MEM_ADDRESS <= mot_mbm_ipu_buffer_address &&
                                        ((u32)fbi->fix.smem_start <= mot_mbm_ipu_buffer_address) &&
                                        ((mot_mbm_ipu_buffer_address + smem_len) <= (MXCIPU_MEM_ADDRESS + MXCIPU_MEM_SIZE))
                                ){

                                        /* ioremap mot_mbm_ipu_buffer_address so that we can relocate the image*/
                                        if (!(io_remapped_logo = ioremap_nocache(mot_mbm_ipu_buffer_address, smem_len))) {
                                                printk("MXCFB - Unable to io-remap logo memory to virtual address: Not relocating\n");
                                        } else {
                                                DPRINTK("ioremaped 0x%08X:to 0x%08X\n", 
                                                                mot_mbm_ipu_buffer_address, 
								(unsigned int)io_remapped_logo);

                                                memcpy_fromio(fbi->screen_base, io_remapped_logo, smem_len);
                                                
                                                DPRINTK("memcpy_fromio(0x%08X, 0x%08X, %d)\n", 
                                                                (unsigned int)fbi->screen_base, 
								(unsigned int)io_remapped_logo, smem_len);
                                                iounmap(io_remapped_logo);
                                                
                                                DPRINTK("Bootlogo relocated to fb0's start address: \n");
                                        }
                                } else {
                                        printk("Bootlogo cannot be relocated to fb0's start address: \n");
                                        printk("\tmot_mbm_ipu_buffer_address=0x%08X IPU-memory: 0x%08lX size=%u\n", 
                                                        mot_mbm_ipu_buffer_address, MXCIPU_MEM_ADDRESS, smem_len);
                                }
                        }
			/* Make sure that the image relocation only happens once */
			mot_mbm_is_ipu_initialized = 0;
                }
#endif

                mxc_fbi->ipu_ch_irq = IPU_IRQ_SDC_BG_EOF;
                mxc_fbi_cli->ipu_ch_irq = IPU_IRQ_SDC_BG_EOF;
                mxc_fbi->ipu_ch = MEM_SDC_BG;
                mxc_fbi_cli->ipu_ch = MEM_SDC_BG;
                ipu_clear_irq(mxc_fbi->ipu_ch_irq);

                if (ipu_sdc_init_panel(mxcfb_panel->type, mxcfb_panel->refresh_rate, 
                                       mxcfb_panel->width, 
				       mxcfb_panel->height + mxcfb_panel->middle_porch_lines,
                                       mxcfb_panel->pixel_fmt,
                                       mxcfb_panel->hStartWidth,
                                       mxcfb_panel->hSyncWidth,
                                       mxcfb_panel->hEndWidth,
                                       mxcfb_panel->vStartWidth,
                                       mxcfb_panel->vSyncWidth,
                                       mxcfb_panel->vEndWidth,
                                       mxcfb_panel->sig_pol) != 0)
                {
                        printk("mxcfb: Error initializing panel.\n");
                        return -EINVAL;
                }
		/* ipu_sdc_set_window_pos() must occur after ipu_sdc_init_panel() */
		ipu_sdc_set_window_pos(mxc_fbi->ipu_ch, 0, 0);
		set_overlay_window();

                ipu_sdc_set_global_alpha(false, 0);
                ipu_sdc_set_color_key(mxc_fbi->ipu_ch, false, 0);

                ipu_init_channel(mxc_fbi->ipu_ch, NULL);
                ipu_init_channel_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER, 
                                        bpp_to_pixfmt(fbi->var.bits_per_pixel),
                                        mxcfb_panel->width, 
					mxcfb_panel->height + mxcfb_panel->middle_porch_lines,
                                        mxcfb_panel->width,
                                        IPU_ROTATE_NONE,
                                        (void*)mxcfb_drv_data.fbi->fix.smem_start, 
                                        (void*)mxcfb_drv_data.fbi->fix.smem_start);

                mxc_fbi->cur_ipu_buf = 0;
                mxc_fbi_cli->cur_ipu_buf = 0;
                ipu_select_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER, 0);

                if (ipu_request_irq(IPU_IRQ_SDC_BG_EOF, mxcfb_irq_handler, 0, 
                                    MXCFB_NAME, fbi) != 0)
                {
                        printk("mxcfb: Error registering irq handler.\n");
                        return -EBUSY;
                }
                ipu_disable_irq(mxc_fbi->ipu_ch_irq);
                ipu_enable_channel(mxc_fbi->ipu_ch);

		retval = mxcfb_blank(FB_BLANK_UNBLANK, fbi);
                if (retval == -ERESTARTSYS) {
			ipu_free_irq(IPU_IRQ_SDC_BG_EOF, fbi);
                        --mxc_fbi_cur->open_count;
                }

        }
        mxc_fbi_cur->open_count++;

        return retval;
}

/*
 * Close the main framebuffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @param       user    Set if opened by user or clear if opened by kernel
 */
static int
mxcfb_release(struct fb_info *fbi, int user)
{
        int retval = 0;
        struct mxcfb_info * mxc_fbi = fbi->par;
	struct partial_mode_info pminfo;

        FUNC_START;

        if(fbi == mxcfb_drv_data.fbi_iram)
                return 0;

	BUG_ON(mxc_fbi->open_count <= 0);
        --mxc_fbi->open_count;

        if (mxc_fbi->open_count == 0) 
        {
#if defined(CONFIG_FB_MXC_OVERLAY)
		if (fbi == mxcfb_drv_data.fbi_ovl) {
			ipu_disable_channel(mxc_fbi->ipu_ch, true);
			ipu_uninit_channel(mxc_fbi->ipu_ch);
			ipu_free_irq(mxc_fbi->ipu_ch_irq, fbi);

		} else {
#else
		{
#endif
			if (((struct mxcfb_info *)mxcfb_drv_data.fbi->par)->open_count == 0 && 
					((struct mxcfb_info *)mxcfb_drv_data.fbi_cli->par)->open_count == 0) {
				pminfo.start_y = 0; pminfo.start_x = 0;
				pminfo.end_y = 0; pminfo.end_x = 0;
				/* 
				 * It's not necessary to protect against partial mode being entered at this point
				 * since this is the last open handle on the framebuffer devices.  Framebuffers
				 * are protected against calling an IOCTL and close function simultaneously.
				 */
				if ((retval = mxcfb_exit_low_power(&pminfo)) == -ERESTARTSYS) {
					++mxc_fbi->open_count;
					return retval;
				}
				retval = mxcfb_blank(FB_BLANK_POWERDOWN, mxcfb_drv_data.fbi); 
				if (retval == -ERESTARTSYS) {
					DPRINTK("MXCFB Failed blank on MAIN, open_count = %d, %s, L.%d\n", 
							mxc_fbi->open_count, __func__, __LINE__);
					++mxc_fbi->open_count;
					return retval;
				}

				retval = mxcfb_blank(FB_BLANK_POWERDOWN, mxcfb_drv_data.fbi_cli); 
				if (retval == -ERESTARTSYS) {
					DPRINTK("MXCFB Failed blank on CLI, open_count = %d, %s, L.%d\n", 
							mxc_fbi->open_count, __func__, __LINE__);
					++mxc_fbi->open_count;
					return retval;
				}

				/* If both main and CLI are blanked, uninit DMA channel */
				ipu_uninit_channel(mxc_fbi->ipu_ch);
				ipu_free_irq(mxc_fbi->ipu_ch_irq, fbi);
			}
		}
        }

        FUNC_END;
        return retval;
}

/*
 * Set fixed framebuffer parameters based on variable settings.
 * 
 * @param       info     framebuffer information pointer
 */
static int mxcfb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)info->par;
	FUNC_START;

	if (mxc_fbi->ipu_ch == MEM_SDC_FG)
		strncpy(fix->id, "DISP3 FG", 8);
	else
		strncpy(fix->id, "DISP3 BG", 8);

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;

	/* 
	 * 1) predefined unique accel number is used by a directfb mxc gfx-driver
	 * to identify fb device it can handle.
	 * 2) gfx driver requires acces to hw registers. fields mmio_start and mmio_len
	 * are used to provide registers mapping.
	 */
	fix->accel = 0x90 ;
	fix->mmio_start  = IPU_CTRL_BASE_ADDR;
	fix->mmio_len    = 0x1BD ;
		
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	return 0;
}

/*
 * Set framebuffer parameters and change the operating mode.
 *
 * @param       info     framebuffer information pointer
 */
static int
mxcfb_set_par(struct fb_info *info)
{
        struct mxcfb_info * mxc_fbi = (struct mxcfb_info *)info->par;

        FUNC_START;

	if (mxcfb_drv_data.suspended == true) {
		printk("MXCFB ERROR: Called %s while suspended\n", __func__);
		return -EBUSY;
	}

	if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
		return -ERESTARTSYS;
	}

        ipu_disable_irq(mxc_fbi->ipu_ch_irq);

	if (info != mxcfb_drv_data.fbi_ovl && panel->prepare_dma_stop_initialize) {
		panel->prepare_dma_stop_initialize();
		msleep(VSYNC_WAIT);
		BUG_ON(!panel->prepare_dma_stop_finalize);
		panel->prepare_dma_stop_finalize();
	}
	ipu_disable_channel(mxc_fbi->ipu_ch, true);
        ipu_clear_irq(mxc_fbi->ipu_ch_irq);
        mxcfb_set_fix(info);

        mxc_fbi->cur_ipu_buf = 0;

	/*
	 * mxcfb_panel points to hvga_panel if the flip is open, or qvga_panel
	 * if the flip is closed
	 */
        ipu_init_channel_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER,
                                bpp_to_pixfmt(info->var.bits_per_pixel),
                                mxcfb_panel->width,
				mxcfb_panel->height + mxcfb_panel->middle_porch_lines,
                                mxcfb_panel->width,
                                IPU_ROTATE_NONE,
                                (void*)mxcfb_drv_data.fbi->fix.smem_start,
                                (void*)mxcfb_drv_data.fbi->fix.smem_start);

        ipu_select_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER, 0);

	if (info == mxcfb_drv_data.fbi_ovl) {
		set_overlay_window();
	} else {
		ipu_sdc_set_window_pos(mxc_fbi->ipu_ch, 0, 0);
		set_overlay_window();
		ipu_enable_channel(mxc_fbi->ipu_ch);
		
		/*
		 * This function is known to cause a flicker on Sharp since we do not
		 * perform the prepare_dma_start during a vsync, but it is not worth 
		 * the risk of modifying the vsync interrupt handler since 
		 * mxcfb_set_par is never called in application code to begin with.
		 */
		if (panel->prepare_dma_start) {
			msleep(1);
			panel->prepare_dma_start();
		}
	}
	up(&mxcfb_global_state.g_sem);
        if(sem_is_locked(&mxc_fbi->flip_sem)) {
                ipu_enable_irq(mxc_fbi->ipu_ch_irq);
        }
        return 0;
}

/*
 * Check framebuffer variable parameters and adjust to valid values.
 * 
 * @param       var      framebuffer variable parameters
 * 
 * @param       info     framebuffer information pointer
 */
static int 
mxcfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	FUNC_START;

	if (var->xres > mxcfb_panel->width)
		var->xres = mxcfb_panel->width;
	if (var->yres > mxcfb_panel->height)
		var->yres = mxcfb_panel->height;
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

#ifdef CONFIG_FB_MXC_INTERNAL_MEM
	if ((var->bits_per_pixel != 24) &&
#else
	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
#endif
	    (var->bits_per_pixel != 16)) {
		var->bits_per_pixel = MXCFB_DEFUALT_BPP;
	}

	switch (var->bits_per_pixel) {
	case 16:
		var->red.length = 5;
#if defined(CONFIG_MOT_FEAT_FB_MXC_RGB)
		var->red.offset = 0;
#else
		var->red.offset = 11;
#endif
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
#if defined(CONFIG_MOT_FEAT_FB_MXC_RGB)
		var->blue.offset = 11;
#else
		var->blue.offset = 0;
#endif
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
#if defined(CONFIG_MOT_FEAT_IPU_BGRA6666)
		/* As the feature name suggests B,G,R and A bitfields' in that 
		 * order are defined. All offsets are from the right,  inside a
		 * "pixel" value, which is exactly 'bits_per_pixel' wide. 
		 * Refer to <LinuxSource>/ include/linux/fb.h for an 
		 * interpretation of offset for color fields.
		 */ 
 		var->red.length = 6;
		var->red.offset = 12;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 6;
		var->green.msb_right = 0;

		var->blue.length = 6;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 6;
		var->transp.offset = 18;
		var->transp.msb_right = 0;
#else /* RGB24 or BGR24 formats */
		var->red.length = 8;
#if defined(CONFIG_MOT_FEAT_FB_MXC_RGB)
		var->red.offset = 0;
#else
		var->red.offset = 16;
#endif
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
#if defined(CONFIG_MOT_FEAT_FB_MXC_RGB)
		var->blue.offset = 16;
#else
		var->blue.offset = 0;
#endif
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
#endif /* MOT_FEAT_IPU_BGRA6666 */ 
		break;
        case 32:
		var->red.length = 8;
#if defined(CONFIG_MOT_FEAT_FB_MXC_RGB)
		var->red.offset = 0;
#else
		var->red.offset = 16;
#endif
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
#if defined(CONFIG_MOT_FEAT_FB_MXC_RGB)
		var->blue.offset = 16;
#else
		var->blue.offset = 0;
#endif
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;
	var->nonstd = 0;

	var->pixclock = -1;
	var->left_margin = -1;
	var->right_margin = -1;
	var->upper_margin = -1;
	var->lower_margin = -1;
	var->hsync_len = -1;
	var->vsync_len = -1;

	var->vmode = FB_VMODE_NONINTERLACED;
	var->sync = 0;
	return 0;

}

static inline u_int _chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}


static int
mxcfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		u_int trans, struct fb_info *fbi)
{
	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 * Y (luminance) =  0.299 * R + 0.587 * G + 0.114 * B 
	 */
	if (fbi->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (fbi->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fbi->pseudo_palette;

			val = _chan_to_field(red, &fbi->var.red);
			val |= _chan_to_field(green, &fbi->var.green);
			val |= _chan_to_field(blue, &fbi->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;

}

/**
 * Evaluates the saved color depths for main and CLI displays and returns
 * the appropriate color depth based on the active panels.  The returned 
 * value is the maximum color depth requested by all active panels.
 *
 * Preconditions: Holding the mxcfb_global_state g_sem mutex is a 
 * prerequisite for calling this function.
 */
static color_depth_t get_current_color_depth(void)
{
    color_depth_t ret_depth = full_color;

    switch (mxcfb_global_state.panel_state) {
	case MAIN_PANEL:
	    ret_depth = mxcfb_global_state.main_color_depth;
	    break;
	case CLI_PANEL:
	    ret_depth = mxcfb_global_state.cli_color_depth;
	    break;
	case (MAIN_PANEL | CLI_PANEL):
	case PANEL_OFF:
	    {
		color_depth_t cli_depth;
		color_depth_t main_depth;

		cli_depth = mxcfb_global_state.cli_color_depth;
		main_depth = mxcfb_global_state.main_color_depth;

		/**
		 * The color depth enum is monotonic with respect to the number of colors.
		 * For example, full_color > low_power_color.
		 * We want to keep the display at the maximum number of requested colors 
		 * for all the active panels.
		 */
		if (cli_depth >= main_depth) {
		    ret_depth = cli_depth;
		} else {
		    ret_depth = main_depth;
		}
	    break;
	    }
	default:
	    printk("Error: Invalid panel_state %d\n", mxcfb_global_state.panel_state);
	    BUG();
    }

    return ret_depth;
}

/* 
 * Turn off the backlight for the HVGA panel.  
 * The backlight is shared between the main and CLI displays, so
 * this method is called by both the MAIN and CLI interfaces.
 *
 * Holding the mxcfb_global_state lock g_sem is a prerequisite 
 * for calling this function.
 */
static void disable_backlight(void)
{
#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD)
        gpio_lcd_backlight_enable(false);
#else
        /* 
	 * Machine type does not have a separate GPIO to control
         * backlight on/off 
	 */
        lights_backlightset(LIGHTS_BACKLIGHT_DISPLAY, 0);

#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD */
        mxcfb_global_state.backlight_state &= ~BACKLIGHT_ON;
}

/* 
 * Turn on the backlight for the HVGA panel.  
 * The backlight is shared between the main and CLI displays, so
 * this function is called by both the MAIN and CLI interfaces.
 *
 * Holding the mxcfb_global_state lock g_sem is a prerequisite 
 * for calling this function.
 */
static void enable_backlight(uint32_t brightness_val)
{
#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD)
        gpio_lcd_backlight_enable(true);
#else
        /* 
	 * Machine type does not have a separate GPIO to control
         * backlight on/off 
	 */
	lights_backlightset(LIGHTS_BACKLIGHT_DISPLAY, 
		                        brightness_val);

#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD */
        mxcfb_global_state.backlight_state |= BACKLIGHT_ON;
}

/* This should be called from a context that holds lock to avoid races */
static int wait_for_vsync(void)
{
        int rc = 0;
        mxcfb_drv_data.vsync_flag = 0;
        ipu_enable_irq(IPU_IRQ_SDC_DISP3_VSYNC);
        rc = wait_event_timeout (mxcfb_drv_data.vsync_wq, mxcfb_drv_data.vsync_flag != 0, VSYNC_WAIT * HZ / 1000);
	if (rc == 0 && !mxcfb_drv_data.vsync_flag) {
                printk("MXCFB_WAIT_FOR_VSYNC: timeout\n");
                rc = -ETIME;
	} else {
                rc = 0;
	}
        return rc;
}


/*
 * Set framebuffer parameters and change the operating mode.
 *
 * Preconditions: Assumes mxcfb_global_state.g_sem lock is held
 */
void hvga_disable_channel(void)
{
        struct mxcfb_info * mxc_fbi;
        struct mxcfb_info * mxc_fbi_cli;

	mxc_fbi = (struct mxcfb_info *)mxcfb_drv_data.fbi->par;
	mxc_fbi_cli = (struct mxcfb_info *)mxcfb_drv_data.fbi_cli->par;

	/* Disable interrupts and the SDC channel DMA */
        ipu_disable_irq(mxc_fbi->ipu_ch_irq);
        ipu_disable_channel(mxc_fbi->ipu_ch, true);
        ipu_clear_irq(mxc_fbi->ipu_ch_irq);

        gpio_ipu_set_pixel_clk(false);
        ndelay(200);
        gpio_lcd_serializer_stby(ASSERT_GPIO_SIGNAL);

        mxc_fbi->cur_ipu_buf = 0;
        mxc_fbi_cli->cur_ipu_buf = 0;
}

#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
static struct timer_list esd_timer;
#endif
/*!
 *  Reconfigure the SDC DMA channel for the appropriate display mode.
 *  The refresh mode and panel state should both be stored in
 *  mxcfb_global_state before calling this function. 
 *
 *  The following modes are valid:
 *
 *   1. Full HVGA DMA mode (HVGA full refresh)
 *   2. Full QVGA refresh on main display, CLI off or in static mode.
 *   3. Full QVGA refresh on CLI display, main off or in static mode.
 *
 *  This function is also used for enabling DMA for one frame to
 *  provide an image update for static mode. See FBIOSTATICIMAGE.
 *
 *  This function assumes that the mxcfb_global_state mutex g_sem is
 *  held as a prerequisite.
 *
 * Preconditions: Assumes mxcfb_global_state.g_sem lock is held
 */
int hvga_reconfigure_channel(void)
{
        int retval = 0;
	struct fb_info * info = NULL;
	struct mxcfb_info * mxc_fbi;
	refresh_mode_t refresh_mode = mxcfb_global_state.refresh_mode;

	/* Disable the SDC DMA channel */
	hvga_disable_channel();

	/*
	 * Determine which refresh mode we're in and reassign mxcfb_panel.
	 * We perform a logical AND on refresh_mode in case more enums are
	 * added in addition to FULL_REFRESH.
	 */
	switch (refresh_mode)
	{
		case REFRESH_OFF:
		    /* Do not re-enable DMA */
		    mxcfb_panel = NULL;
#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
                    if(timer_pending(&esd_timer)) {
                        del_timer_sync(&esd_timer);
                    }
#endif
                    ipu_disable_sdc();
		    break;
		case MAIN_FULL_REFRESH:
		    mxcfb_panel = panel->qvga_panel_info;
		    info = mxcfb_drv_data.fbi;
		    break;
		case CLI_FULL_REFRESH:
		    mxcfb_panel = panel->qvga_panel_info;
                    if(mxcfb_global_state.fb_flip_status == FB_FLIP_OPEN)
                            info = mxcfb_drv_data.fbi_iram;
                    else if(mxcfb_global_state.fb_flip_status == FB_FLIP_CLOSE)
                            info = mxcfb_drv_data.fbi_cli;
		    break;
		case (MAIN_FULL_REFRESH | CLI_FULL_REFRESH):
                    DPRINTK("***: HVGA: { (MAIN_FULL_REFRESH | CLI_FULL_REFRESH) AND (fb_flip_status=%d) }\n", (mxcfb_global_state.fb_flip_status));
                    mxcfb_panel = panel->hvga_panel_info;
                    info = mxcfb_drv_data.fbi;
		    break;
		default:
		    return -EINVAL;
		    break;
	}

	/*
	 * mxcfb_panel points to hvga_panel if the flip is open, or qvga_panel
	 * if the flip is closed. If NULL, then the panel/refresh is entirely off,
	 * and we have no more work to do.
	 */
	if (!mxcfb_panel) {
		return retval;
	}

        mxc_fbi = (struct mxcfb_info *)info->par;
        ipu_enable_sdc();

        gpio_lcd_serializer_stby(DEASSERT_GPIO_SIGNAL);
        /* /STBY to active-edge of Strobe tdValid */
        udelay(30);
        gpio_ipu_set_pixel_clk(true);

	/*
	 * Reconfigure the DMA channel for either HVGA or QVGA, with an updated
	 * smem_start if needed
	 */
        ipu_init_channel_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER,
                                bpp_to_pixfmt(info->var.bits_per_pixel),
                                mxcfb_panel->width,
				mxcfb_panel->height + mxcfb_panel->middle_porch_lines,
                                mxcfb_panel->width,
                                IPU_ROTATE_NONE,
                                (void*)info->fix.smem_start,
                                (void*)info->fix.smem_start);

	if (ipu_sdc_init_panel(mxcfb_panel->type, mxcfb_panel->refresh_rate, 
				mxcfb_panel->width, 
				mxcfb_panel->height + mxcfb_panel->middle_porch_lines,
				mxcfb_panel->pixel_fmt,
				mxcfb_panel->hStartWidth,
				mxcfb_panel->hSyncWidth,
				mxcfb_panel->hEndWidth,
				mxcfb_panel->vStartWidth,
				mxcfb_panel->vSyncWidth,
				mxcfb_panel->vEndWidth,
				mxcfb_panel->sig_pol) != 0)
	{
		printk("mxcfb: Error initializing panel.\n");
		return -EINVAL;
	}

	/* ipu_sdc_set_window_pos() must occur after ipu_sdc_init_panel() */
	ipu_sdc_set_window_pos(mxc_fbi->ipu_ch, 0, 0);

	/*
	 * The DMA channel is configured for double buffering (for easy switching
	 * of the smem_start address), so select the first buffer.
	 */
        ipu_select_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER, 0);
	ipu_enable_channel(mxc_fbi->ipu_ch);

#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
        if(!timer_pending(&esd_timer)) {
                mod_timer(&esd_timer, jiffies + esd_delay);
        }
#endif

        return retval;
}

/* 
 * Causes the panel to enter low power partial mode state (mode 6).
 * The PM suspend/resume routines are unused in this driver due to 
 * the hacks necessary for DSM 1 minute wakeups, so FBIOPARTIALMODE
 * uses this function to enter the suspended state instead.
 */
static int mxcfb_set_low_power(struct partial_mode_info * pm_info)
{
	int retval = 0;

	if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
		return -ERESTARTSYS;
	}

	/* Main panel must be turned off before entering partial mode */
	if (mxcfb_global_state.panel_state & MAIN_PANEL) {
		retval = -EINVAL;
		goto err;
	}

	mxcfb_global_state.partial_coords = *pm_info;

	if (mxcfb_drv_data.suspended == false) {
		/* Set panel low power mode */
		mxcfb_global_state.refresh_mode = REFRESH_OFF;
		mxcfb_global_state.static_image_mode_state |= CLI_STATIC_MODE;
		mxcfb_drv_data.suspended = true;
		panel->enter_low_power_mode(pm_info);
	} else {
		/* Update panel low power window */
		panel->update_low_power_mode(pm_info);
	}

        set_overlay_window();
err:
	up(&mxcfb_global_state.g_sem);
	return retval;
}

/* 
 * Causes the panel to exit low power partial mode state (mode 6).
 * The PM suspend/resume routines are unused in this driver due to 
 * the hacks necessary for DSM 1 minute wakeups, so FBIOPARTIALMODE
 * uses this function to exit the suspended state instead.
 */
static int mxcfb_exit_low_power(struct partial_mode_info * pm_info)
{
	if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
		return -ERESTARTSYS;
	}

	if (mxcfb_drv_data.suspended == false) {
		/* Nothing to do, already resumed */
		up(&mxcfb_global_state.g_sem);
		return 0;
	}

	mxcfb_drv_data.suspended = false;
	mxcfb_global_state.refresh_mode = CLI_FULL_REFRESH;
	mxcfb_global_state.static_image_mode_state &= ~CLI_STATIC_MODE;
	mxcfb_global_state.partial_coords = *pm_info;
	panel->exit_low_power_mode();

        set_overlay_window();

	wait_for_vsync();

	up(&mxcfb_global_state.g_sem);

	return 0;
}


/*
 * Changes the current static mode state.  If static mode is entered on a 
 * panel, then RGB refresh is stopped on that interface.  When static mode
 * is exited, then RGB refresh is resumed on the interface.  
 * 
 *  panelsrc: Must be either MAIN_PANEL or CLI_PANEL
 *  static_mode: whether to enter or exit static mode
 *  Returns: 0 on success, -EINVAL or -ERESTARTSYS on error
 */
int mxcfb_set_static_mode(panel_t panelsrc, static_state_t static_mode)
{
	refresh_mode_t curr_refresh_mode;
	refresh_mode_t full_refresh_mode;
	static_image_mode_state_t new_static_mode;

	curr_refresh_mode = mxcfb_global_state.refresh_mode;

	if (panelsrc == MAIN_PANEL) {
		full_refresh_mode = MAIN_FULL_REFRESH;
		new_static_mode = MAIN_STATIC_MODE;
	} else if (panelsrc == CLI_PANEL) {
		full_refresh_mode = CLI_FULL_REFRESH;
		new_static_mode = CLI_STATIC_MODE;
	} else {
		return -EINVAL;
	}

	if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
		return -ERESTARTSYS;
	}

	switch (static_mode) {
		case ENTER_STATIC_MODE:
			if (new_static_mode & mxcfb_global_state.static_image_mode_state) {
				break;
			}
			curr_refresh_mode &= ~full_refresh_mode;
			mxcfb_global_state.static_image_mode_state |= new_static_mode;
			panel->set_refresh_mode(curr_refresh_mode);
			break;
		case EXIT_STATIC_MODE:
                        if(panelsrc == CLI_PANEL && mxcfb_global_state.fb_flip_status == FB_FLIP_OPEN) {
                                DPRINTK("***EXIT static mode on CLI when FLIP-open\n");
                                break;
                        }
			if (!(new_static_mode & mxcfb_global_state.static_image_mode_state)) {
				break;
			}
			curr_refresh_mode |= full_refresh_mode;
			mxcfb_global_state.static_image_mode_state &= ~new_static_mode;
			panel->set_refresh_mode(curr_refresh_mode);
			break;
		case UPDATE_STATIC_IMAGE:
			if (!(new_static_mode & mxcfb_global_state.static_image_mode_state)) {
				break;
			}
			curr_refresh_mode |= full_refresh_mode;
			panel->set_refresh_mode(curr_refresh_mode);
			/* Need to wait for 2 VSYNC interrupts to guarantee a refresh */
			wait_for_vsync();
			wait_for_vsync();
			curr_refresh_mode &= ~full_refresh_mode;
			panel->set_refresh_mode(curr_refresh_mode);
			break;
		default:
			up(&mxcfb_global_state.g_sem);
			return -EINVAL;
	}

        set_overlay_window();
	up(&mxcfb_global_state.g_sem);

	return 0;
}

/* 
 * Sets the DSM partial window.  If the DSM partial window start and end 
 * coordinates are both 0, then this function will exit partial mode and
 * panel low power mode (mode 6).  If, on the other hand, one of the 
 * y-coordinate values is non-zero, then this function will enter
 * a low power state and suspend the framebuffer driver.  The static image
 * in the partial window can be updated by calling this function subsequent
 * times.
 */
static int mxcfb_set_partial_mode(struct partial_mode_info * pm_info)
{
	int retval = 0; 

	/* Verify that the coordinates passed are within the QVGA limits */
	if(pm_info->start_y >= pm_info->end_y ||
			pm_info->end_y > (panel->qvga_panel_info->height - 1) || 
			pm_info->start_y < 0) {

		/* If all offsets are 0, then exit partial mode */
		if (pm_info->start_y == 0 && pm_info->end_y == 0 
				&& pm_info->start_x == 0 && pm_info->end_x == 0) {

			/* Exit partial mode, low power mode */
			retval = mxcfb_exit_low_power(pm_info);
		} else {
			/* An invalid partial mode has been passed in */
			retval = -EINVAL;
		}
	} else {
		retval = mxcfb_set_low_power(pm_info);
	}

	return retval;
}

int mxcfb_flip_status(fb_flip_state_t flip)
{
        int rc = 0;
	struct mxcfb_info * mxc_fbi_ovl = mxcfb_drv_data.fbi_ovl->par;

        FUNC_START;

        if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
                return -ERESTARTSYS;
        }

        if(flip == mxcfb_global_state.fb_flip_status) {
                up(&mxcfb_global_state.g_sem);
                return 0;
        }


        mxcfb_global_state.fb_flip_status = flip;
        if(mxcfb_global_state.fb_flip_status == FB_FLIP_OPEN) {

                if( (mxcfb_global_state.partial_coords.start_y != 0 ||
                                mxcfb_global_state.partial_coords.end_y != 0) ) {
                        mxcfb_global_state.partial_coords.start_x = 0;
                        mxcfb_global_state.partial_coords.start_y = 0;
                        mxcfb_global_state.partial_coords.end_x = 0;
                        mxcfb_global_state.partial_coords.end_y = 0;
                        mxcfb_drv_data.suspended = false;
                        panel->exit_low_power_mode();
                }

                mxcfb_global_state.overlay_panel = MAIN_PANEL;
                panel->set_refresh_mode(CLI_FULL_REFRESH);

                wait_for_vsync();
                wait_for_vsync();
                
		panel->switch_panel_state(MAIN_PANEL|CLI_PANEL, MAIN_FULL_REFRESH);
                mxcfb_global_state.static_image_mode_state = CLI_STATIC_MODE;

		if(mxc_fbi_ovl->open_count > 0){
                        ipu_sdc_set_global_alpha(true, 255);
                }

                set_overlay_window();

                enable_backlight(mxcfb_global_state.user_brightness_pref);

	} else if(mxcfb_global_state.fb_flip_status == FB_FLIP_CLOSE) {
		if( mxcfb_global_state.static_image_mode_state & MAIN_STATIC_MODE )
		{
			mxcfb_global_state.static_image_mode_state &= ~MAIN_STATIC_MODE ;
			panel->set_refresh_mode(mxcfb_global_state.refresh_mode | MAIN_FULL_REFRESH);
		}

		mxcfb_global_state.overlay_panel = CLI_PANEL;
		mxcfb_global_state.static_image_mode_state = NO_STATIC_MODE;
		panel->switch_panel_state(CLI_PANEL, CLI_FULL_REFRESH);

		if(mxc_fbi_ovl->open_count > 0){
			ipu_sdc_set_global_alpha(true, 255);
		}

		set_overlay_window();

	} else {
                DPRINTK("INVALID Flip status\n");
                rc = -EINVAL;
        }

        up(&mxcfb_global_state.g_sem);
        FUNC_END;
        return rc;
}

/*
 * Function to handle custom ioctls for MXC framebuffer.
 *
 * @param       inode   inode struct
 *
 * @param       file    file struct
 *
 * @param       cmd     Ioctl command to handle
 *
 * @param       arg     User pointer to command arguments
 *
 * @param       fbi     framebuffer information pointer
 */
static int mxcfb_ioctl(struct inode *inode, struct file *file,
		       unsigned int cmd, unsigned long arg, struct fb_info *fbi)
{
	int retval = 0;
	struct mxcfb_info * mxc_fbi_ovl = mxcfb_drv_data.fbi_ovl->par;

	if (cmd == MXCFB_WAIT_FOR_VSYNC || cmd == FBIOSET_COLOR_DEPTH || cmd == FBIOSTATICIMAGE || 
			cmd == FBIOROTATEIMAGE || cmd == FBIOSWAP_BUFFERS || cmd == FBIOSET_OVERLAY_PANEL) {
		/* The suspended variable is set by FBIOPARTIALMODE */
		if (mxcfb_drv_data.suspended == true) {
			printk("MXCFB ERROR: Called %s while suspended\n", __func__);
			return -EBUSY;
		}
	}


	switch (cmd) {
#if defined(CONFIG_FB_MXC_OVERLAY)
		case MXCFB_SET_GBL_ALPHA:
			{
				struct mxcfb_gbl_alpha ga;
				if (copy_from_user(&ga, (void *)arg, sizeof(ga))) {
					retval = -EFAULT;
					break;
				}
				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
			                return -ERESTARTSYS;
			        }

				if (mxc_fbi_ovl->open_count>0)

					retval = ipu_sdc_set_global_alpha((bool) ga.enable,ga.alpha);
				else
					DPRINTK("Overlay is not open\n");

				up(&mxcfb_global_state.g_sem);			
	
				DPRINTK("Set global alpha to %d\n", ga.alpha);
				break;
			}
		case MXCFB_SET_CLR_KEY:
			{
				struct mxcfb_color_key key;
				if (copy_from_user(&key, (void *)arg, sizeof(key))) {
					retval = -EFAULT;
					break;
				}
				retval = ipu_sdc_set_color_key(MEM_SDC_BG, key.enable,
						key.color_key);
				DPRINTK("Set color key to 0x%08X\n", key.color_key);
				break;
			}
#endif
		case MXCFB_WAIT_FOR_VSYNC:
			retval = wait_for_vsync();
			DPRINTK("MXCFB_WAIT_FOR_VSYNC: retval = %d\n", retval);
			break;
		case MXCFB_SET_BRIGHTNESS:
			{
				uint8_t level;
				if (copy_from_user(&level, (void *)arg, sizeof(level))) {
					retval = -EFAULT;
					break;
				}
#if defined(CONFIG_MACH_MXC27530EVB) || defined(CONFIG_MACH_I30030EVB) \
				|| defined(CONFIG_MACH_MXC91131EVB)
				mxcfb_drv_data.backlight_level = level;
				retval = ipu_sdc_set_brightness(level);
				DPRINTK("Set brightness to %d\n", level);
#else
				retval = -EINVAL;
#endif
				break;
			}
		case FBIOPARTIALMODE:
			{
				struct partial_mode_info pm_info; 

				if (fbi != mxcfb_drv_data.fbi_cli) {
					DPRINTK("MXCFB Error: Partial Mode only supported on CLI PANEL\n");
					retval = -EINVAL;
					break;
				}

				if(copy_from_user(&pm_info, (void __user *)arg, sizeof(struct partial_mode_info))) {
					retval = -EFAULT;
					break;
				}

				retval = mxcfb_set_partial_mode(&pm_info);

				break;
			}
		case FBIOSET_COLOR_DEPTH:
			{
				if (arg < 0 || arg > full_color) {
					retval = -EINVAL;
					break;
				}

				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					return -ERESTARTSYS;
				}
				if (fbi == mxcfb_drv_data.fbi) {
					mxcfb_global_state.main_color_depth = arg;
				} else if (fbi == mxcfb_drv_data.fbi_cli) {
					mxcfb_global_state.cli_color_depth = arg;
				} else {
					retval = -EINVAL;
					up(&mxcfb_global_state.g_sem);
					break;
				}
				mxcfb_global_state.curr_color_depth = get_current_color_depth();
				panel->set_color_depth(mxcfb_global_state.curr_color_depth);
				up(&mxcfb_global_state.g_sem);
				break;
			}
		case FBIOSTATICIMAGE:
			{
				DPRINTK("MXCFB setting Static Mode: arg = %lu\n", arg);

				if (arg < 0 || arg > UPDATE_STATIC_IMAGE) {
					retval = -EINVAL;
					break;
				}

				if (fbi == mxcfb_drv_data.fbi) {
					retval = mxcfb_set_static_mode(MAIN_PANEL, arg);
				} else if (fbi == mxcfb_drv_data.fbi_cli) {
					retval = mxcfb_set_static_mode(CLI_PANEL, arg);
				} else {
					retval = -EINVAL;
				}

				break;
			}
		case FBIOROTATEIMAGE:
			{
				if (arg < 0 || arg > IMAGE_ROTATE_180) {
					retval = -EINVAL;
					break;
				}
				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					return -ERESTARTSYS;
				}
				if (fbi == mxcfb_drv_data.fbi) {
					mxcfb_global_state.main_rotation_value = arg;
				} else if (fbi == mxcfb_drv_data.fbi_cli) {
					mxcfb_global_state.cli_rotation_value = arg;
				} else {
					retval = -EINVAL;
				}
				panel->rotate_image(mxcfb_global_state.main_rotation_value, 
						mxcfb_global_state.cli_rotation_value);
				up(&mxcfb_global_state.g_sem);
				break;
			}
		case FBIOSWAP_BUFFERS:
			{
				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					return -ERESTARTSYS;
				}
				if (mxcfb_global_state.panel_state == (MAIN_PANEL | CLI_PANEL)) {
					mxcfb_global_state.buffers_swapped = 1 - mxcfb_global_state.buffers_swapped;
					panel->swap_buffers();
				} else {
					retval = -EINVAL;
				}
				up(&mxcfb_global_state.g_sem);
				break;
			}
		case FBIOGET_CURRENT_PANEL:
			{
				unsigned long curr_panel;

				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					return -ERESTARTSYS;
				}

				if (fbi == mxcfb_drv_data.fbi) {
					curr_panel = (mxcfb_global_state.buffers_swapped == 1) ? CLI_PANEL : MAIN_PANEL;
				} else if (fbi == mxcfb_drv_data.fbi_cli) {
					curr_panel = (mxcfb_global_state.buffers_swapped == 1) ? MAIN_PANEL : CLI_PANEL;
#if defined(CONFIG_FB_MXC_OVERLAY)
				} else if (fbi == mxcfb_drv_data.fbi_ovl) {
					curr_panel = mxcfb_global_state.overlay_panel;
#endif
				} else {
					retval = -EINVAL;
					up(&mxcfb_global_state.g_sem);
					break;
				}
				retval = copy_to_user((unsigned long *)arg, &curr_panel, sizeof(unsigned long));
				up(&mxcfb_global_state.g_sem);
				break;
			}
#if defined(CONFIG_FB_MXC_OVERLAY)
		case FBIOSET_OVERLAY_PANEL:
			{
				/* Verify that a valid parameter was passed in */
				if (arg != MAIN_PANEL && arg != CLI_PANEL) {
					return -EINVAL;
				}
				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					return -ERESTARTSYS;
				}
                                if( ((mxcfb_global_state.fb_flip_status == FB_FLIP_OPEN) && (arg != MAIN_PANEL)) ||
                                                ((mxcfb_global_state.fb_flip_status == FB_FLIP_CLOSE) && (arg != CLI_PANEL)) ) {
                                        up(&mxcfb_global_state.g_sem);
                                        return -EINVAL;
                                }
				/* Try to set the overlay to the requested panel */
                                if(arg != mxcfb_global_state.overlay_panel) {
                                        mxcfb_global_state.overlay_panel = arg;
                                        set_overlay_window();
                                }
				up(&mxcfb_global_state.g_sem);
				break;
			}
#endif
		case FBIOSETBKLIGHT:
			{
				DPRINTK("mxcfb_ioctl:FBIOSETBKLIGHT,arg(%ld)\n",arg);
				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					retval = -ERESTARTSYS;
					break;
				}
				switch (arg) {
					case BKLIGHT_OFF:
						disable_backlight();
						break;
					case BKLIGHT_ON:
						enable_backlight(mxcfb_global_state.brightness);
						break;
					default:
						retval = -EINVAL;
				}
				up(&mxcfb_global_state.g_sem);
				break;
			}
		case FBIOGETBKLIGHT:
			{
				unsigned int i;

				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					return -ERESTARTSYS;
				}
#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD)
				i = gpio_get_lcd_backlight();
#else
				i = (mxcfb_global_state.backlight_state & BACKLIGHT_ON) != 0;
#endif
				up(&mxcfb_global_state.g_sem);

				retval = put_user(i, (unsigned int __user *)arg);
				break;
			}
                case FBIO_STORE_USER_BRIGHTNESS_PREF:
                                DPRINTK("store_brightness=%d\n", arg);
                                if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
                                        retval = -ERESTARTSYS;
                                        break;
                                }

                                if(fbi == mxcfb_drv_data.fbi) {
                                        mxcfb_global_state.user_brightness_pref = arg;
                                }else {
                                        retval = -EINVAL;
                                }

                                up(&mxcfb_global_state.g_sem);
                                break;
		case FBIOSETBRIGHTNESS:
			{
				if (arg > mxcfb_global_state.bklight_range.max || 
						arg < mxcfb_global_state.bklight_range.min)
				{
					retval = -EINVAL;
					break;
				}
				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					retval = -ERESTARTSYS;
					break;
				}
				mxcfb_global_state.brightness = arg;
#if defined(CONFIG_MACH_MXC27530EVB) || defined(CONFIG_MACH_I30030EVB) \
				|| defined(CONFIG_MACH_MXC91131EVB)
				if ((retval = ipu_sdc_set_brightness(arg)) != 0) {
					up(&mxcfb_global_state.g_sem);
					retval = -EFAULT;
					break;
				}
#elif defined(CONFIG_MACH_ARGONLVREF)
				pwm_set_lcd_bkl_brightness(mxcfb_global_state.brightness);	
#else	
#if defined(CONFIG_MACH_ASCENSION) || defined(CONFIG_MACH_LIDO) \
				|| defined(CONFIG_MACH_SAIPAN)
				/* Do not change the brightness if the backlight state is currently off */
				if (mxcfb_global_state.backlight_state & BACKLIGHT_ON) {
#endif
					lights_backlightset(LIGHTS_BACKLIGHT_DISPLAY,
							mxcfb_global_state.brightness);
#if defined(CONFIG_MACH_ASCENSION) || defined(CONFIG_MACH_LIDO) \
					|| defined(CONFIG_MACH_SAIPAN)
				}
#endif
#endif /* defined(CONFIG_MACH_ARGONLVREF) */
				up(&mxcfb_global_state.g_sem);
				break;
			}
		case FBIOGETBRIGHTNESS:
			{
				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					return -ERESTARTSYS;
				}
#if defined(CONFIG_MACH_ARGONLVREF)
				mxcfb_global_state.brightness = pwm_get_lcd_bkl_brightness();	
#endif
				retval = copy_to_user((unsigned long *)arg, 
						&(mxcfb_global_state.brightness), 
						sizeof(unsigned long));
				up(&mxcfb_global_state.g_sem);
				retval = retval ? -EFAULT : 0;
				break;
			}
		case FBIOSET_BRIGHTNESSRANGE:
			{
				struct backlight_brightness_range bklight_range;
				if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
					return -ERESTARTSYS;
				}
				if (copy_from_user(&bklight_range, (void *)arg, 
							sizeof(bklight_range))) 
				{
					up(&mxcfb_global_state.g_sem);
					retval = -EFAULT;
					break;
				}
				mxcfb_global_state.bklight_range = bklight_range;
				up(&mxcfb_global_state.g_sem);
				break;
			}
#if defined(CONFIG_MOT_FEAT_IPU_IOCTL_EZX_COMPAT) 
		case FBIOENABLE2BFS:
			{
				mxcfb_global_state.dbuffer_enabled = DBUFFER_ENABLED;
				break;
			}
		case FBIODISABLE2BFS:
			{
				int i;
				struct fb_var_screeninfo var = mxcfb_drv_data.fbi->var;
				var.yoffset = 0;
				acquire_console_sem();
				i = fb_pan_display(mxcfb_drv_data.fbi, &var);
				release_console_sem();

				if (i) 
				{
					retval = i;
					break;
				}
				mxcfb_global_state.dbuffer_enabled = DBUFFER_DISABLED;
				break;
			}
		case FBIOCHECK2BFS:
			{
				unsigned long screen_addr;
				retval = (mxcfb_global_state.dbuffer_enabled == DBUFFER_ENABLED);
				if (retval)
				{
					screen_addr = mxcfb_drv_data.fbi->fix.smem_start + 
						mxcfb_drv_data.fbi->var.yres * mxcfb_drv_data.fbi->var.xres * 
						( mxcfb_drv_data.fbi->var.bits_per_pixel / 8 );

					if (copy_to_user((unsigned long *)arg, &screen_addr, 
								sizeof(unsigned long)) != 0)
					{
						retval = -EFAULT;
					}
				}
				break;
			}
		case FBIOCKMAINVALIDFB:
			{
				retval = 1;
				break;
			}
#endif /* defined(CONFIG_MOT_FEAT_IPU_IOCTL_EZX_COMPAT) */
#if defined(CONFIG_MOT_FEAT_EMULATED_CLI)
		case FBIOENABLE_EMULATEDCLI:
		case FBIODISABLE_EMULATEDCLI:
			/* Return success by default */
			break;
#endif /* defined(CONFIG_MOT_FEAT_EMULATED_CLI) */
#if defined(CONFIG_MOT_FEAT_IPU_IOCTL)
		case FBIO_QUERY_DISPLAY_TYPE:
			{
				unsigned long disp_type = TRANSMISSIVE_DISPLAY; /* default to TRANSMISSIVE */

				if(fbi == mxcfb_drv_data.fbi)
#if defined(CONFIG_FB_MXC_MAIN_TRANSFLECTIVE_DISPLAY)
					disp_type = TRANSFLECTIVE_DISPLAY;
#else
				disp_type = TRANSMISSIVE_DISPLAY;
#endif
				else if(fbi == mxcfb_drv_data.fbi_cli)
#if defined(CONFIG_FB_MXC_CLI_TRANSFLECTIVE_DISPLAY)
					disp_type = TRANSFLECTIVE_DISPLAY;
#else
				disp_type = TRANSMISSIVE_DISPLAY;
#endif
				else {
					retval = -EINVAL;
					break;
				}
				retval = copy_to_user((unsigned long *)arg, &disp_type, sizeof(unsigned long));
				break;
			}
#endif /* CONFIG_MOT_FEAT_IPU_IOCTL */
		default:
			retval = -EINVAL;
	}
	return retval;
}

#ifdef CONFIG_FB_MXC_OVERLAY
/*
 * Open the overlay framebuffer.
 * 
 * @param       fbi     framebuffer information pointer
 * 
 * @param       user    Set if opened by user or clear if opened by kernel
 */
static int 
mxcfb_ovl_open(struct fb_info *fbi, int user)
{
        struct mxcfb_info * mxc_fbi = (struct mxcfb_info *)fbi->par;

        FUNC_START;

	if (mxcfb_drv_data.suspended == true) {
		printk("MXCFB ERROR: Called %s while suspended\n", __func__);
		return -EBUSY;
	}

        if (mxc_fbi->open_count == 0)
        {
                mxc_fbi->ipu_ch_irq = IPU_IRQ_SDC_FG_EOF;
                mxc_fbi->ipu_ch = MEM_SDC_FG;
                ipu_clear_irq(mxc_fbi->ipu_ch_irq);

                ipu_init_channel(mxc_fbi->ipu_ch, NULL);
                ipu_init_channel_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER, 
                                        bpp_to_pixfmt(fbi->var.bits_per_pixel),
                                        fbi->var.xres, fbi->var.yres,
                                        fbi->var.xres_virtual,
                                        IPU_ROTATE_NONE,
                                        (void*)fbi->fix.smem_start, 
                                        (void*)fbi->fix.smem_start);
                mxc_fbi->cur_ipu_buf = 0;

                ipu_select_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER, 0);

                if (ipu_request_irq(mxc_fbi->ipu_ch_irq, mxcfb_irq_handler, 0, 
                                    MXCFB_NAME, fbi) != 0)
                {
                        printk("mxcfb: Error registering irq handler.\n");
			ipu_uninit_channel(mxc_fbi->ipu_ch);
                        return -EBUSY;
                }
                ipu_disable_irq(mxc_fbi->ipu_ch_irq);

		if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
			ipu_uninit_channel(mxc_fbi->ipu_ch);
			ipu_free_irq(mxc_fbi->ipu_ch_irq, fbi);
			return -ERESTARTSYS;
		}
                mxc_fbi->open_count++;
                ipu_sdc_set_window_pos(mxc_fbi->ipu_ch, 0, 0);
		set_overlay_window();
		up(&mxcfb_global_state.g_sem);
        } else {
                mxc_fbi->open_count++;
        }
        return 0;
}
#endif


/*
 * This function does not take the g_sem lock and directly
 * goes about doing what is asked for, instead of making
 * decisions based on the flip-status
 * */
static int do_blank(int blank, struct fb_info *fbi)
{
        struct mxcfb_info * mxc_fbi = (struct mxcfb_info *)mxcfb_drv_data.fbi->par;
	panel_t new_state;
	panel_t old_state;
	refresh_mode_t new_refresh_mode;
	color_depth_t old_color_depth;

        FUNC_START;
        DPRINTK("blank = %d\n", blank);

	if (mxcfb_drv_data.suspended == true) {
		printk("MXCFB ERROR: Called %s while suspended\n", __func__);
		return -EBUSY;
	}

	if (fbi == mxcfb_drv_data.fbi_ovl) {
	    /* Blank is not defined for the video overlay */
	    return -EINVAL;
	}

	mxc_fbi->blank = blank;

	new_refresh_mode = mxcfb_global_state.refresh_mode;
	old_state = mxcfb_global_state.panel_state;
	new_state = old_state;

        switch (blank) {
                case FB_BLANK_POWERDOWN:
                case FB_BLANK_VSYNC_SUSPEND:
                case FB_BLANK_HSYNC_SUSPEND:
                case FB_BLANK_NORMAL:
			if (fbi == mxcfb_drv_data.fbi) {
				new_state &= ~MAIN_PANEL;
				new_refresh_mode &= ~MAIN_FULL_REFRESH;
			} else if (fbi == mxcfb_drv_data.fbi_cli) {
				new_state &= ~CLI_PANEL;
				new_refresh_mode &= ~CLI_FULL_REFRESH;
			} else {
				goto err;
			}
                        break;
                case FB_BLANK_UNBLANK:
			if (fbi == mxcfb_drv_data.fbi) {
				new_state |= MAIN_PANEL;
				if (!(mxcfb_global_state.static_image_mode_state & MAIN_STATIC_MODE)) {
					new_refresh_mode |= MAIN_FULL_REFRESH;
				}
			} else if (fbi == mxcfb_drv_data.fbi_cli) {
				new_state |= CLI_PANEL;
				if (!(mxcfb_global_state.static_image_mode_state & CLI_STATIC_MODE)) {
					new_refresh_mode |= CLI_FULL_REFRESH;
				}
			} else {
				goto err;
			}
                        break;
        }

	if (old_state == new_state) {
		return 0;
	}

        panel->switch_panel_state(new_state, new_refresh_mode);

	/* Check to see if the color mode needs updating */
	old_color_depth = mxcfb_global_state.curr_color_depth;
	mxcfb_global_state.curr_color_depth = get_current_color_depth();
	if (mxcfb_global_state.curr_color_depth != old_color_depth) {
                panel->set_color_depth(mxcfb_global_state.curr_color_depth);
	}

        return 0;
err:
	BUG();
	return -EINVAL;
}

/*
 * mxcfb_blank():
 *      Blank the display.
 */
static int mxcfb_blank(int blank, struct fb_info *fbi)
{
        int rc;

        if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
            return -ERESTARTSYS;
        }

        rc = do_blank(blank, fbi);

        up(&mxcfb_global_state.g_sem);
        return rc;
}

/*
 * do_ioctl_blank performs blanking as performed by the FBIOBLANK ioctl.
 * This implements some blanking logic based on the state of the flip
 * to avoid going into HVGA refresh on keypress wakeup.
 */
static int
do_ioctl_blank(int blank, struct fb_info *fbi)
{
        int rc = 0;

        /* Lock the mxcfb_global_state variable */
        if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
            return -ERESTARTSYS;
        }

        if(mxcfb_global_state.fb_flip_status == FB_FLIP_OPEN &&
           fbi == mxcfb_drv_data.fbi) {
                if(blank == FB_BLANK_UNBLANK &&
                   mxcfb_global_state.panel_state == PANEL_OFF) {
                        /* Display wakeup.  Unblank the CLI first, update logo,
                         * put CLI into static mode, and unblank main.
                         */
                        panel->switch_panel_state(CLI_PANEL, CLI_FULL_REFRESH);
                        wait_for_vsync();
                        wait_for_vsync();
                        panel->switch_panel_state(MAIN_PANEL|CLI_PANEL, MAIN_FULL_REFRESH);
                        mxcfb_global_state.static_image_mode_state = CLI_STATIC_MODE;
                        enable_backlight(mxcfb_global_state.brightness);
                } else if (blank != FB_BLANK_UNBLANK) {
                        /* Main is being turned off with the flip open.
                         * Turn CLI off, too.
                         */
                        rc = do_blank(blank, fbi);
                        rc = do_blank(blank, mxcfb_drv_data.fbi_cli);
                }
        }

        set_overlay_window();

        up(&mxcfb_global_state.g_sem);
        return rc;
}

/*
 * mxcfb_fbmem_blank():
 *      Blank the display as called by the fbmem framework.  Blanking is
 *      really expensive, and fbmem is holding the console semaphore while
 *      this runs.  Since we already have a lock to prevent races within
 *      this driver, we release the console semaphore while waiting for
 *      the display to switch the reacquire before returning.
 */
static int mxcfb_fbmem_blank(int blank, struct fb_info *fbi)
{
        int rc;

        release_console_sem();
        rc = do_ioctl_blank(blank, fbi);
        acquire_console_sem();

        return rc;
}

/*
 * Pan or Wrap the Display
 *
 * This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
 *
 * @param               var     Variable screen buffer information
 * @param               info    Framebuffer information pointer
 */
static int
mxcfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
        struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)info->par;
        uint32_t lock_flags = 0;
        u_int y_bottom;
        unsigned long base;

        FUNC_START;

	if (mxcfb_drv_data.suspended == true) {
		printk("MXCFB ERROR: Called %s while suspended\n", __func__);
		return -EBUSY;
	}

        if (var->xoffset > 0) {
                DPRINTK("x panning not supported\n");
                return -EINVAL;
        }

        if ((info->var.xoffset == var->xoffset) &&
            (info->var.yoffset == var->yoffset)) {
                return 0; // No change, do nothing
        }

        y_bottom = var->yoffset;

        if (!(var->vmode & FB_VMODE_YWRAP)) {
                y_bottom += var->yres;
        }

        if (y_bottom > info->var.yres_virtual) {
                return -EINVAL;
        }

        base = (var->yoffset * var->xres_virtual + var->xoffset);
        base *= (var->bits_per_pixel) / 8;
        base += info->fix.smem_start;

        ipu_clear_irq(mxc_fbi->ipu_ch_irq);
        ipu_enable_irq(mxc_fbi->ipu_ch_irq);
        down(&mxc_fbi->flip_sem);

        spin_lock_irqsave(&mxc_fbi->fb_lock, lock_flags);

        mxc_fbi->cur_ipu_buf = !mxc_fbi->cur_ipu_buf;
        if (ipu_update_channel_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER,
                mxc_fbi->cur_ipu_buf, (void*)base) == 0) {
                ipu_select_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER,
                                  mxc_fbi->cur_ipu_buf);
        }
        else {
                printk("Error updating SDC buf %d to address=0x%08X\n",
                        mxc_fbi->cur_ipu_buf, (uint32_t)base);
        }

        spin_unlock_irqrestore(&mxc_fbi->fb_lock, lock_flags);

        info->var.xoffset = var->xoffset;
        info->var.yoffset = var->yoffset;

        if (var->vmode & FB_VMODE_YWRAP) {
                info->var.vmode |= FB_VMODE_YWRAP;
        } else {
                info->var.vmode &= ~FB_VMODE_YWRAP;
        }

        return 0;
}

#ifdef CONFIG_MOT_FEAT_DISABLE_SW_CURSOR
static int disable_sw_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	return 0;
}
#endif

/*!
 * This structure contains the pointers to the control functions that are 
 * invoked by the core framebuffer driver to perform operations like 
 * blitting, rectangle filling, copy regions and cursor definition.
 */
static struct fb_ops mxcfb_ops = {
        .owner                          = THIS_MODULE,
        .fb_open                        = mxcfb_open,
        .fb_release                     = mxcfb_release,
        .fb_set_par                     = mxcfb_set_par,
        .fb_check_var                   = mxcfb_check_var,
        .fb_setcolreg                   = mxcfb_setcolreg,
        .fb_pan_display                 = mxcfb_pan_display,
        .fb_fillrect                    = cfb_fillrect,
        .fb_copyarea                    = cfb_copyarea,
        .fb_imageblit                   = cfb_imageblit,
        .fb_blank                       = mxcfb_fbmem_blank,
        .fb_ioctl                       = mxcfb_ioctl,
#ifdef CONFIG_MOT_FEAT_DISABLE_SW_CURSOR
        .fb_cursor                      = disable_sw_cursor,
#else
        .fb_cursor                      = soft_cursor,
#endif
};

#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
static struct workqueue_struct * esd_workqueue;

/*
 * Work queue handler for ESD (electrostatic discharge) recovery.  
 * This function puts the serializer into standby and pulls it out, 
 * since ESD has been shown to force the deserializer into standby.
 * Deasserting STBY on the serializer will also deassert STBY on the 
 * deserializer.
 */
static void esd_work_handler(void * data)
{
        if(panel->prepare_dma_start) {
                msleep(1);
		panel->prepare_dma_start();
        }
	up(&mxcfb_global_state.g_sem);
	mxcfb_drv_data.esd_flag = 0;
	mod_timer(&esd_timer, jiffies + esd_delay);
}
static DECLARE_WORK(esd_work, esd_work_handler, NULL);


/* 
 * The main ESD handling routine, which is called from the VSYNC interrupt. 
 * The commands here must be executed during the VSYNC back porch, so
 * they must be executed as quickly as possible after the vsync.  
 *
 * NOTE: this function may be run in interrupt context. Do not use *sleep. 
 */
static void esd_fixup(void)
{
	if (panel->prepare_dma_stop_finalize) {
		panel->prepare_dma_stop_finalize();
	}

#if defined(CONFIG_MOT_FEAT_IPU_IOCTL)
	gpio_ipu_set_pixel_clk(false);
#endif
	ndelay(200);
	gpio_lcd_serializer_stby(ASSERT_GPIO_SIGNAL);
	udelay(25);
	gpio_lcd_serializer_stby(DEASSERT_GPIO_SIGNAL);
	udelay(30);
#if defined(CONFIG_MOT_FEAT_IPU_IOCTL)
	gpio_ipu_set_pixel_clk(true);
#endif
	
	if (panel->prepare_dma_start) {
		/* workqueue to handle time-insensitive commands that sleep */
		queue_work(esd_workqueue, &esd_work);
	} else {
		up(&mxcfb_global_state.g_sem);
		mxcfb_drv_data.esd_flag = 0;
		mod_timer(&esd_timer, jiffies + esd_delay);
	}
}

/* 
 * ESD timer handler to register for VSYNC interrupts.  When the vsync 
 * interrupt is generated and the esd_flag is set, then a workqueue will
 * be started to execute the ESD recovery routine.
 */
static void esd_timer_handler(unsigned long private)
{
	if(down_trylock(&mxcfb_global_state.g_sem) != 0) {
		/* reschedule in 1 second */
		mod_timer(&esd_timer, jiffies + HZ);
		return;
	}
	if (panel->prepare_dma_stop_initialize) {
		panel->prepare_dma_stop_initialize();
	}
	mxcfb_drv_data.esd_flag = 1;
	ipu_clear_irq(IPU_IRQ_SDC_DISP3_VSYNC);
	ipu_enable_irq(IPU_IRQ_SDC_DISP3_VSYNC);
	/* Do NOT release lock until esd_work_handler */
}
#endif /* defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY) */

/* 
 * IPU SDC VSYNC interrupt.  Triggers when the VSYNC line to the parallel
 * SDC interface is toggled.
 */
static irqreturn_t mxcfb_vsync_irq_handler(int irq, void *dev_id, 
                                           struct pt_regs *regs)
{
        struct mxcfb_data * fb_data = dev_id;

        ipu_disable_irq(irq);
	
#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
	if (mxcfb_drv_data.esd_flag) {
		esd_fixup();
	}
#endif /* defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY) */

        fb_data->vsync_flag = 1;
        wake_up_interruptible(&fb_data->vsync_wq);

        return IRQ_HANDLED;
}

/* 
 * This IPU End of Frame interrupt handler should be removed, but it is 
 * harmless and deemed too risky to remove at this stage.  
 */
static irqreturn_t mxcfb_irq_handler(int irq, void *dev_id, 
                                     struct pt_regs *regs)
{
        struct fb_info * fbi = dev_id;
        struct mxcfb_info * mxc_fbi = fbi->par;

        up(&mxc_fbi->flip_sem);
        ipu_disable_irq(irq);
        return IRQ_HANDLED;
}

static int mxcfb_map_iram_memory(struct fb_info *fbi)
{

        fbi->fix.smem_len = SZ_256K;
        fbi->fix.smem_start = 0x78000000;

        DPRINTK("allocated fb @ paddr=0x%08X, size=%d.\n",
                (uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);
#ifdef CONFIG_MOT_FEAT_IPU_MEM_ADDR
        if (!ipu_dynamic_pool && !request_mem_region(fbi->fix.smem_start, fbi->fix.smem_len, "LCD")) {
#else
        if (!request_mem_region(fbi->fix.smem_start, fbi->fix.smem_len, "LCD")) {
#endif /* CONFIG_MOT_FEAT_IPU_MEM_ADDR */

                return -EBUSY;
        }

        if (!(fbi->screen_base = ioremap(fbi->fix.smem_start,
                                         fbi->fix.smem_len))) {
                release_mem_region(fbi->fix.smem_start, fbi->fix.smem_len);
                printk("MXCFB - Unable to map fb memory to virtual address\n");
                return -EIO;
        }
        fbi->screen_size = fbi->fix.smem_len;

        return 0;
}

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped 
 * into a non-cached, non-buffered, memory region to allow palette and pixel 
 * writes to occur without flushing the cache.  Once this area is remapped, 
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 * 
 * @return      Error code indicating success or failure
 * */
static int
mxcfb_map_video_memory(struct fb_info *fbi)
{
	unsigned long fbsize = 0;
	unsigned long middle_porch_len = 0;

	/* Framebuffer length for QVGA displays */
	fbi->fix.smem_len = (def_vram == 0) ? fbi->var.xres_virtual *
#if defined(CONFIG_MOT_WFN394)
	    fbi->var.yres_virtual * (fbi->var.bits_per_pixel >> 3) : def_vram;
#else
	fbi->var.yres_virtual * 4 : def_vram;
#endif

	/* Dummy data that must be DMA'd between main and CLI displays */
	middle_porch_len = mxcfb_drv_data.fbi->var.xres_virtual * 
		(mxcfb_drv_data.fbi->var.bits_per_pixel >> 3) * panel->hvga_panel_info->middle_porch_lines;

	/* 
	 * Only allocate HVGA memory for the first framebuffer device since the main and CLI 
	 * framebuffers share the same DMA channel.
	 */
        if (fbi == mxcfb_drv_data.fbi) {
		    fbsize = fbi->fix.smem_len * 2 + middle_porch_len;
	} 
#if defined(CONFIG_FB_MXC_OVERLAY)
	else if (fbi == mxcfb_drv_data.fbi_ovl) {
		    fbsize = fbi->fix.smem_len;
	}
#endif
        
        if (fbi != mxcfb_drv_data.fbi_cli) {
                fbi->fix.smem_start = ipu_malloc(fbsize);

                if (fbi->fix.smem_start == 0) {
                        printk("MXCFB - Unable to allocate framebuffer memory\n");
                        return -EBUSY;
                }
                DPRINTK("mxcfb: allocated fb @ paddr=0x%08X, size=%d.\n", 
                        (uint32_t)fbi->fix.smem_start, fbi->fix.smem_len);
#ifdef CONFIG_MOT_FEAT_IPU_MEM_ADDR
                if (!ipu_dynamic_pool && !request_mem_region(fbi->fix.smem_start, fbsize, "LCD")) {
#else
                if (!request_mem_region(fbi->fix.smem_start, fbsize, "LCD")) {
#endif /* CONFIG_MOT_FEAT_IPU_MEM_ADDR */        
                        return -EBUSY;
                }

                if (!(fbi->screen_base = ioremap_nocache(fbi->fix.smem_start, fbsize))) {
                        release_mem_region(fbi->fix.smem_start, fbsize);
                        printk("MXCFB - Unable to map fb memory to virtual address\n");
                        return -EIO;
                }
        } else {
                fbi->screen_base = mxcfb_drv_data.fbi->screen_base + 
			mxcfb_drv_data.fbi->screen_size + middle_porch_len;
                fbi->fix.smem_start = mxcfb_drv_data.fbi->fix.smem_start + 
			mxcfb_drv_data.fbi->fix.smem_len + middle_porch_len;
        }
        fbi->screen_size = fbi->fix.smem_len;

#ifndef CONFIG_MOT_FEAT_POWERUP_LOGO
        /** 
	 * Clear the screen only if IPU was not initialized by MBM 
         * Doing this will cause the logo be cleared 
         * This may not happen if IRAM is used for fb0, but other platforms
         * can be affected
         *
         * Prevent this only for the main framebuffer device(fb0)
         * NB: mxcfb_drv_data.fbi is set by the time this function is called
         */
        if(fbi != mxcfb_drv_data.fbi) {
	    /* Clear the screen */
	    memset((char*)fbi->screen_base, 0, fbi->fix.smem_len);
	}
#else /* CONFIG_MOT_FEAT_POWERUP_LOGO */
#if defined(CONFIG_FB_MXC_OVERLAY)
	/** 
	 * With powerup graphics on a HVGA display, we only want to 
	 * clear the overlay, since a branding logo is being displayed
	 * on both the main and CLI panels.
	 */
	if (fbi == mxcfb_drv_data.fbi_ovl) {
	    memset((char*)fbi->screen_base, 0, fbi->fix.smem_len);
	}
#endif
#endif /* CONFIG_MOT_FEAT_POWERUP_LOGO */


        return 0;
}


/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int
mxcfb_unmap_video_memory(struct fb_info *fbi)
{
	unsigned long middle_porch_len;

	if (fbi != mxcfb_drv_data.fbi_cli) {
	        iounmap(fbi->screen_base);
		if (fbi == mxcfb_drv_data.fbi) {
			middle_porch_len = mxcfb_drv_data.fbi->var.xres_virtual * 
				(mxcfb_drv_data.fbi->var.bits_per_pixel >> 3) * 
				(mxcfb_panel->middle_porch_lines);
			release_mem_region(fbi->fix.smem_start, 
				fbi->fix.smem_len * 2 + middle_porch_len);
		} else {
			release_mem_region(fbi->fix.smem_start, fbi->fix.smem_len);
		}
                ipu_free(fbi->fix.smem_start);
	}

        return 0;
}

/*!
 * Initializes the framebuffer information pointer. After allocating
 * sufficient memory for the framebuffer structure, the fields are
 * filled with custom information passed in from the configurable
 * structures.  This includes information such as bits per pixel,
 * color maps, screen width/height and RGBA offsets.
 *
 * @return      Framebuffer structure initialized with our information
 */
static struct fb_info * 
#if defined(CONFIG_FB_MXC_OVERLAY)
mxcfb_init_fbinfo(struct device *dev, struct fb_ops * ops, int ovl)
#else
mxcfb_init_fbinfo(struct device *dev, struct fb_ops * ops)
#endif
{
        struct fb_info *fbi;
        struct mxcfb_info *mxcfbi;

        /*
         * Allocate sufficient memory for the fb structure
         */
        fbi = framebuffer_alloc(sizeof(struct mxcfb_info), dev);
        if (!fbi)
                return NULL;

        mxcfbi = (struct mxcfb_info *)fbi->par;

         /*
         * Fill in fb_info structure information
         */
        fbi->var.xres = fbi->var.xres_virtual = panel->qvga_panel_info->width;
        fbi->var.yres = fbi->var.yres_virtual = panel->qvga_panel_info->height;
#if defined(CONFIG_FB_MXC_OVERLAY)
	if (ovl) /* Overlay should always be double buffered */
	    fbi->var.yres_virtual *= 2;
#endif
        fbi->var.activate = FB_ACTIVATE_NOW;
        mxcfb_check_var(&fbi->var, fbi);

        mxcfb_set_fix(fbi);

        fbi->fbops                   = ops;
        fbi->flags                   = FBINFO_FLAG_DEFAULT;
        fbi->pseudo_palette          = mxcfbi->pseudo_palette;

        spin_lock_init(&mxcfbi->fb_lock);
        sema_init(&mxcfbi->flip_sem, 0);

        /*
         * Allocate colormap
         */
        fb_alloc_cmap(&fbi->cmap, 16, 0);

        return fbi;
}

#if defined(CONFIG_MOT_FEAT_DISABLE_SW_CURSOR)
static void dont_flash_cursor(void *priv)
{
        /*
         * This is a dummy function so that fbconsole
         * doesnt try to use its timer-based cursor flashing function.
         * This function should not get called
         */
        BUG();
}
#endif

/* Initialize the mxcfb_global_state variable to default values */
static void init_global_state(void)
{
	init_MUTEX(&mxcfb_global_state.g_sem);
#ifdef CONFIG_MOT_FEAT_POWERUP_LOGO
	if (mot_mbm_is_ipu_initialized) {
		mxcfb_global_state.panel_state = MAIN_PANEL | CLI_PANEL;
		mxcfb_global_state.refresh_mode = MAIN_FULL_REFRESH | CLI_FULL_REFRESH;
	} else {
#else /* !CONFIG_MOT_FEAT_POWERUP_LOGO */
	mxcfb_global_state.panel_state = PANEL_OFF;
	mxcfb_global_state.refresh_mode = REFRESH_OFF;
#endif
#ifdef CONFIG_MOT_FEAT_POWERUP_LOGO
	}
#endif
	mxcfb_global_state.overlay_panel = MAIN_PANEL;
	mxcfb_global_state.static_image_mode_state = NO_STATIC_MODE;
	mxcfb_global_state.curr_color_depth = full_color;
	mxcfb_global_state.main_color_depth = full_color;
	mxcfb_global_state.cli_color_depth = full_color;
	mxcfb_global_state.buffers_swapped = 0;
	mxcfb_global_state.brightness = DEFAULT_BRIGHTNESS;
	mxcfb_global_state.backlight_state = BACKLIGHT_ON;
	mxcfb_global_state.bklight_range.min = MIN_BRIGHTNESS;
	mxcfb_global_state.bklight_range.max = MAX_BRIGHTNESS;
	mxcfb_global_state.partial_coords.start_x = 0;
	mxcfb_global_state.partial_coords.start_y = 0;
	mxcfb_global_state.partial_coords.end_x = 0;
	mxcfb_global_state.partial_coords.end_y = 0;
        mxcfb_global_state.fb_flip_status = FB_FLIP_OPEN;
	mxcfb_global_state.main_rotation_value = IMAGE_ROTATE_0;
	mxcfb_global_state.cli_rotation_value = IMAGE_ROTATE_0;
#if defined(CONFIG_MOT_FEAT_IPU_IOCTL_EZX_COMPAT)
	mxcfb_global_state.dbuffer_enabled = DBUFFER_ENABLED;
#endif
}

static void mxcfb_reset_serializer(void)
{
	gpio_lcd_serializer_reset(ASSERT_GPIO_SIGNAL);
	/* 10ms delay required for Sharp powerup (1ms for E02) */
	msleep(10);
	gpio_lcd_serializer_reset(DEASSERT_GPIO_SIGNAL);
	/* 20ms delay required for E02 reset sequence (10ms for Sharp) */
	msleep(20);
}

static int
powerdown(struct notifier_block *self, unsigned long reboot_type, void *dummy)
{
        if (reboot_type == SYS_HALT || reboot_type == SYS_POWER_OFF) {
                /* Blank the CLI so a residual image doesn't decay off
                 * the screen when we lose power.
                 */
                mxcfb_blank(FB_BLANK_POWERDOWN, mxcfb_drv_data.fbi_cli);
        }

        return NOTIFY_OK;
}

static struct notifier_block powerdown_notifier = {
        .notifier_call = powerdown,
};

static int
flipkeyd_notify(struct notifier_block *self, unsigned long ignored,
                void *event)
{
        mpm_event_t *ev = (mpm_event_t*)event;

        if (ev->type == MPM_EVENT_DEVICE) {
                if (ev->kind == EVENT_DEV_FLIP) {
                        notifier_flip_state = (ev->info == DEV_ON ?
                                               FB_FLIP_OPEN : FB_FLIP_CLOSE);
                        wake_up_process(flipkeyd_task);
                } else if (ev->kind == EVENT_DEV_KEY) {
                        notifier_key_pressed = 1;
                        wake_up_process(flipkeyd_task);
                }
        }

        return NOTIFY_OK;
}

static struct notifier_block flipkeyd_notifier = {
        .notifier_call = flipkeyd_notify,
};

/*!
 * Kernel thread for processing display driver changes for flip or key events.
 * To speed up respose to the flip event, this kthread is woken up and
 * issues the flip status change.  This is a relatively cheap thing to do
 * in terms of CPU utilization but expensive in terms of time, so this
 * context pays the time cost while regular processes can continue running.
 */
static int flipkeyd(void *dummy)
{
        struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO - 1 };

        __set_current_state(TASK_RUNNING);
        sched_setscheduler(current, SCHED_FIFO, &param);
        notifier_chain_register(&mpm_flipkey_notifier_list,
                                &flipkeyd_notifier);

        do {
                mxcfb_flip_status(notifier_flip_state);

                if (notifier_key_pressed) {
                        notifier_key_pressed = 0;

                        if (mxcfb_global_state.fb_flip_status == FB_FLIP_OPEN) {
                                /* Keypress to wakeup both displays */
                                do_ioctl_blank(FB_BLANK_UNBLANK, mxcfb_drv_data.fbi);
                        }
                }

                set_current_state(TASK_INTERRUPTIBLE);
                if (mxcfb_global_state.fb_flip_status == notifier_flip_state
                    && !notifier_key_pressed) {
                        schedule();
                }
                __set_current_state(TASK_RUNNING);

        } while (!kthread_should_stop());

        notifier_chain_unregister(&mpm_flipkey_notifier_list,
                                  &flipkeyd_notifier);
        return 0;
}

/*!
 * Probe routine for the framebuffer driver. It is called during the
 * driver binding process.      The following functions are performed in
 * this routine: Framebuffer initialization, Memory allocation and
 * mapping, Framebuffer registration, IPU initialization.
 *
 * @return      Appropriate error code to the kernel common code
 */
static int __init mxcfb_probe(struct device *dev)
{
        struct fb_info *fbi, *fbi_cli, *fbi_ovl, *fbi_iram;
        struct mxcfb_info *mxc_fbi, *mxc_fbi_cli, *mxc_fbi_ovl, *mxc_fbi_iram;
        int ret;
#if defined(CONFIG_MOT_FEAT_POWERUP_LOGO)
	/*
	 * mot_mbm_is_ipu_initialized may be modified by mxcfb_set_par before we've
	 * finished mxcfb_probe due to the console driver opening fb0 as a framebuffer
	 * console when it's registered.
	 */
	u32 temp_mot_mbm_is_ipu_initialized = mot_mbm_is_ipu_initialized;
#endif

        FUNC_START;

	init_global_state();

#if defined(CONFIG_MOT_FEAT_POWERUP_LOGO)
	/*
	 * Initialize the ADC SPI interface
	 */
	if (mot_mbm_is_ipu_initialized)
	{
		setup_panel_pointers();
		mxcfb_init_serial_display();
	}
	else
#endif
	{
		/*
		 * Initialize timings to E02 by default. Since both
		 * E02 and Sharp share the same ADC parameters, the
		 * serial interface does not need to be reconfigured
		 * for Sharp.
		 */
		panel = &E02_panel_functions;
		mxcfb_panel = panel->hvga_panel_info;

		mxcfb_reset_serializer();
		mxcfb_init_serial_display();
		setup_panel_pointers();
	}

#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
	init_timer(&esd_timer);
	esd_timer.data = (unsigned long)NULL;
	esd_timer.function = esd_timer_handler;
	esd_timer.expires = jiffies + esd_delay;
#endif

        /*
         * Initialize FB structures
         */
#if defined(CONFIG_FB_MXC_OVERLAY)
        fbi = mxcfb_init_fbinfo(dev, &mxcfb_ops, 0);
#else
        fbi = mxcfb_init_fbinfo(dev, &mxcfb_ops);
#endif
        if (!fbi) {
                ret = -ENOMEM;
                goto err0;
        }
#if defined(CONFIG_MOT_FEAT_DISABLE_SW_CURSOR)
        /*
         * Define a dummy function so that fbconsole doesnt try to use
         * its own timer-based cursor flashing function
         * This is to disable fbconsole displaying software cursor
         */
        fbi->queue.func = dont_flash_cursor;
#endif

#if defined(CONFIG_FB_MXC_OVERLAY)
        fbi_ovl = mxcfb_init_fbinfo(dev, &mxcfb_ops, 1);
        if (!fbi_ovl) {
                ret = -ENOMEM;
                goto err1;
        }
#if defined(CONFIG_MOT_FEAT_DISABLE_SW_CURSOR)
        /*
         * Define a dummy function so that fbconsole doesnt try to use
         * its own timer-based cursor flashing function
         * This is to disable fbconsole displaying software cursor
         */
        fbi_ovl->queue.func = dont_flash_cursor;
#endif
#else /* !defined(CONFIG_FB_MXC_OVERLAY) */
	fbi_ovl = NULL;
#endif /* defined(CONFIG_FB_MXC_OVERLAY) */

#if defined(CONFIG_FB_MXC_OVERLAY)
        fbi_cli = mxcfb_init_fbinfo(dev, &mxcfb_ops, 0);
#else
        fbi_cli = mxcfb_init_fbinfo(dev, &mxcfb_ops);
#endif
        if(!fbi_cli) {
                ret = -ENOMEM;
                goto err2;
        }
#if defined(CONFIG_MOT_FEAT_DISABLE_SW_CURSOR)
        /*
         * Define a dummy function so that fbconsole doesnt try to use
         * its own timer-based cursor flashing function
         * This is to disable fbconsole displaying software cursor
         */
        fbi_cli->queue.func = dont_flash_cursor;
#endif

#if defined(CONFIG_FB_MXC_OVERLAY)
        fbi_iram = mxcfb_init_fbinfo(dev, &mxcfb_ops, 0);
#else
        fbi_iram = mxcfb_init_fbinfo(dev, &mxcfb_ops);
#endif
        if(!fbi_iram) {
                ret = -ENOMEM;
                goto err2_iram;
        }

        /*
         * Set up the mxcfb_drv_data global structure
         */
        mxc_fbi = fbi->par;
        mxcfb_drv_data.fbi = fbi;

#if defined(CONFIG_FB_MXC_OVERLAY)
        mxc_fbi_ovl = fbi_ovl->par;
        mxcfb_drv_data.fbi_ovl = fbi_ovl;
#else
	mxc_fbi_ovl = NULL;
	mxcfb_drv_data.fbi_ovl = NULL;
#endif

        mxc_fbi_cli = fbi_cli->par;
        mxcfb_drv_data.fbi_cli = fbi_cli;

        mxc_fbi_iram = fbi_iram->par;
        mxc_fbi_iram->ipu_ch_irq = IPU_IRQ_SDC_BG_EOF;
        mxc_fbi_iram->ipu_ch = MEM_SDC_BG;
        mxc_fbi_iram->cur_ipu_buf = 0;
        mxcfb_drv_data.fbi_iram = fbi_iram;


        ret = mxcfb_map_video_memory(fbi);
        if (ret < 0) {
                ret = -ENOMEM;
                goto err3;
        }
	/*
	 * fbi_cli memory must be allocated before fbi_ovl since we want the CLI
	 * framebuffer memory to directly follow the main display framebuffer
	 * memory.
	 */
        ret = mxcfb_map_video_memory(fbi_cli);
        if(ret < 0) {
                ret = -ENOMEM;
                goto err4;
        }
#if defined(CONFIG_FB_MXC_OVERLAY)
        ret = mxcfb_map_video_memory(fbi_ovl);
        if (ret < 0) {
                ret = -ENOMEM;
                goto err5;
        }
#endif

        ret = mxcfb_map_iram_memory(fbi_iram);
        if(ret<0) {
                ret = -ENOMEM;
                goto err5_iram;
        }

        /* copy the fb2 memory to fbiram */
        memcpy_fromio(fbi_iram->screen_base, fbi_cli->screen_base, fbi_cli->fix.smem_len);

        DPRINTK("Done mxcfb_map_video_memory on all devices\n");

        /*
         * Register overlay framebuffer
         */
        ret = register_framebuffer(fbi);
        if (ret < 0) {
                goto err6;
        }
        DPRINTK("Done register_framebuffer: fbi\n");
#if defined(CONFIG_FB_MXC_OVERLAY)
        ret = register_framebuffer(fbi_ovl);
        if (ret < 0) {
                goto err7;
        }
        DPRINTK("Done register_framebuffer: fbi_ovl\n");
#endif
        ret = register_framebuffer(fbi_cli);
        if (ret < 0) {
                goto err8;
        }
        DPRINTK("Done register_framebuffer: fbi_cli\n");

        ret = register_framebuffer(fbi_iram);
        if(ret < 0) {
                return -ENOMEM;
                goto err8_iram;
        }
        DPRINTK("Done with register fbi_iram\n");

        init_waitqueue_head(&mxcfb_drv_data.vsync_wq);
        init_waitqueue_head(&mxcfb_drv_data.sys1_eof_wq);
        if ((ret = ipu_request_irq(IPU_IRQ_SDC_DISP3_VSYNC,
                mxcfb_vsync_irq_handler,
                0, MXCFB_NAME, &mxcfb_drv_data)) < 0) {
                goto err9;
        }
        ipu_disable_irq(IPU_IRQ_SDC_DISP3_VSYNC);

	if ((ret = ipu_request_irq(IPU_IRQ_ADC_SYS1_EOF, mxcfb_sys1_eof_irq_handler, 0,
				MXCFB_NAME, fbi)) != 0) {
		printk("mxcfb: Error registering ADC_SYS1 irq handler.\n");
		goto err10;
	}
	ipu_disable_irq(IPU_IRQ_ADC_SYS1_EOF);

        dev_set_drvdata(dev, &mxcfb_drv_data);
        DPRINTK("%s registered\n", MXCFB_NAME);

#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
	esd_workqueue = create_singlethread_workqueue("LCD_ESD");
	mxcfb_drv_data.esd_flag = 0;
	esd_timer.expires = jiffies + esd_delay; 
	add_timer(&esd_timer); 
#endif

#if defined(CONFIG_MOT_FEAT_POWERUP_LOGO)
	/* Reset and reinitialize the panel if it is not configured by MBM */
	if (!temp_mot_mbm_is_ipu_initialized)
#endif
	{
		mxcfb_reset_serializer();
		panel->init_panel();
	}

	if (panel->init_channel_template) {
		panel->init_channel_template();
	}

        mxcfb_global_state.static_image_mode_state = CLI_STATIC_MODE;
        panel->set_refresh_mode(mxcfb_global_state.refresh_mode & ~CLI_FULL_REFRESH);

        wake_up_process(flipkeyd_task);
        register_reboot_notifier(&powerdown_notifier);

        FUNC_END;
        return 0;

err10:
	ipu_clear_irq(IPU_IRQ_ADC_SYS1_EOF);
err9:
        unregister_framebuffer(fbi_iram);
err8_iram:
        unregister_framebuffer(fbi_cli);
err8:
#if defined(CONFIG_FB_MXC_OVERLAY)
        unregister_framebuffer(fbi_ovl);
err7:
#endif
        unregister_framebuffer(fbi);
err6:
        mxcfb_unmap_video_memory(fbi_iram);
err5_iram:
#if defined(CONFIG_FB_MXC_OVERLAY)
        mxcfb_unmap_video_memory(fbi_ovl);
err5:
#endif
        mxcfb_unmap_video_memory(fbi_cli);
err4:
        mxcfb_unmap_video_memory(fbi);
err3:
        if (fbi_iram) {
                fb_dealloc_cmap(&fbi_iram->cmap);
		framebuffer_release(fbi_iram);
	}
err2_iram:
        if (fbi_cli) {
                fb_dealloc_cmap(&fbi_cli->cmap);
		framebuffer_release(fbi_cli);
	}
err2:
#if defined(CONFIG_FB_MXC_OVERLAY)
        if (fbi_ovl) {
                fb_dealloc_cmap(&fbi_ovl->cmap);
		framebuffer_release(fbi_ovl);
	}
err1:
#endif
        if (fbi) {
                fb_dealloc_cmap(&fbi->cmap);
		framebuffer_release(fbi);
	}
err0:
        return ret;
}

#ifdef CONFIG_PM
/*
 * Power management hooks.      Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */

/*
 * Suspends the framebuffer and blanks the screen. Power management support
 */
static int mxcfb_suspend(struct device *dev, u32 state, u32 level)
{
        FUNC_START;
        DPRINTK("level = %d\n", level);

        FUNC_END;
        return 0;
}

/*
 * Resumes the framebuffer and unblanks the screen. Power management support
 */
static int mxcfb_resume(struct device *dev, u32 level)
{

        FUNC_START;
        DPRINTK("level = %d\n", level);

        FUNC_END;
        return 0;
}
#else
#define mxcfb_suspend   NULL
#define mxcfb_resume    NULL
#endif

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct device_driver mxcfb_driver = {
        .name                   = MXCFB_NAME,
        .bus                    = &platform_bus_type,
        .probe                  = mxcfb_probe,
        .suspend                = mxcfb_suspend,
        .resume                 = mxcfb_resume,
};

/*!
 * Device definition for the Framebuffer
 */
static struct platform_device mxcfb_device = {
        .name                   = MXCFB_NAME,
        .id                     = 0,
};


#ifdef MXCFB_DEBUG

/*
 * Set up "panel_state" as a driver attribute.
 */
static ssize_t show_panel_state(struct device_driver *drv, char *buf)
{
	panel_t current_state;

	current_state = mxcfb_global_state.panel_state;
	return sprintf(buf, "Main Display: %s\nCLI Display: %s\n\n", 
			(current_state & MAIN_PANEL) ? "ON" : "OFF", 
			(current_state & CLI_PANEL) ? "ON" : "OFF");
}

static int parse_panel_state(const char *buf, size_t count, panel_t * mode, bool * state)
{
#define MAIN_ON "main on"
#define MAIN_OFF "main off"
#define CLI_ON "cli on"
#define CLI_OFF "cli off"

	int retval = 0;

	if (strnicmp(buf, MAIN_ON, sizeof(MAIN_ON) / sizeof(char) - 1) == 0) {
		*mode = MAIN_PANEL;
		*state = true;
	} else if (strnicmp(buf, MAIN_OFF, sizeof(MAIN_OFF) / sizeof(char) - 1) == 0) {
		*mode = MAIN_PANEL;
		*state = false;
	} else if (strnicmp(buf, CLI_ON, sizeof(CLI_ON) / sizeof(char) - 1) == 0) {
		*mode = CLI_PANEL;
		*state = true;
	} else if (strnicmp(buf, CLI_OFF, sizeof(CLI_OFF) / sizeof(char) - 1) == 0) {
		*mode = CLI_PANEL;
		*state = false;
	} else {
		retval = -EINVAL;
	}

	return retval;
}

static ssize_t store_panel_state(struct device_driver *drv, const char *buf, size_t count) 
{
	int retval;
	panel_t mode;
	bool panel_on;

	if (parse_panel_state(buf, count, &mode, &panel_on) < 0) {
		return -EINVAL;
	}

	if (mode == MAIN_PANEL) {
	    if (panel_on) {
		retval = mxcfb_blank(FB_BLANK_UNBLANK, mxcfb_drv_data.fbi);
	    } else {
		retval = mxcfb_blank(FB_BLANK_POWERDOWN, mxcfb_drv_data.fbi);
	    }
	} 
	else {
	    if (panel_on) {
		retval = mxcfb_blank(FB_BLANK_UNBLANK, mxcfb_drv_data.fbi_cli);
	    } else {
		retval = mxcfb_blank(FB_BLANK_POWERDOWN, mxcfb_drv_data.fbi_cli);
	    }
	}

	return (retval < 0) ? retval : count;
}
static DRIVER_ATTR(panel_state, S_IRUSR|S_IWUSR, show_panel_state, store_panel_state);

/*
 * Set up "static_mode" as a driver attribute.
 */
static ssize_t show_static_mode(struct device_driver *drv, char *buf)
{
	return sprintf(buf, "Main Display Static Mode: %s\nCLI Display Static Mode: %s\n\n",
			(mxcfb_global_state.static_image_mode_state & MAIN_STATIC_MODE) ? "ON":"OFF",
			(mxcfb_global_state.static_image_mode_state & CLI_STATIC_MODE) ? "ON":"OFF");
}

static ssize_t store_static_mode(struct device_driver *drv, const char *buf, size_t count) 
{
	int retval;
	panel_t mode;
	bool panel_on;

	if (parse_panel_state(buf, count, &mode, &panel_on) < 0) {
		return -EINVAL;
	}

	if (mode == MAIN_PANEL) {
	    if (panel_on) {
		retval = mxcfb_ioctl(NULL, NULL, FBIOSTATICIMAGE, ENTER_STATIC_MODE, mxcfb_drv_data.fbi);
	    } else {
		retval = mxcfb_ioctl(NULL, NULL, FBIOSTATICIMAGE, EXIT_STATIC_MODE, mxcfb_drv_data.fbi);
	    }
	} 
	else {
	    if (panel_on) {
		retval = mxcfb_ioctl(NULL, NULL, FBIOSTATICIMAGE, ENTER_STATIC_MODE, mxcfb_drv_data.fbi_cli);
	    } else {
		retval = mxcfb_ioctl(NULL, NULL, FBIOSTATICIMAGE, EXIT_STATIC_MODE, mxcfb_drv_data.fbi_cli);
	    }
	}
	return (retval < 0) ? retval : count;
}
static DRIVER_ATTR(static_mode, S_IRUSR|S_IWUSR, show_static_mode, store_static_mode);

/*
 * Set up "swap_buffers" as a driver attribute.
 */
static ssize_t show_swap_buffers(struct device_driver *drv, char *buf)
{
	return sprintf(buf, "Main and CLI displays are %sswapped\n", 
			mxcfb_global_state.buffers_swapped ? "" : "NOT ");
}

static ssize_t store_swap_buffers(struct device_driver *drv, const char *buf, size_t count) 
{
	int retval;

	retval = mxcfb_ioctl(NULL, NULL, FBIOSWAP_BUFFERS, 0, mxcfb_drv_data.fbi);

	return (retval < 0) ? retval : count;
}
static DRIVER_ATTR(swap_buffers, S_IRUSR|S_IWUSR, show_swap_buffers, store_swap_buffers);

static ssize_t store_spi_refresh(struct device_driver *drv, const char *buf, size_t count) 
{
	if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
		return -ERESTARTSYS;
	}
	mxcfb_update_spi_frame();
	up(&mxcfb_global_state.g_sem);

	return count;
}
static DRIVER_ATTR(spi_refresh, S_IRUSR|S_IWUSR, NULL, store_spi_refresh);


static ssize_t store_partial_window(struct device_driver *drv, const char *buf, size_t count) 
{
	int retval = 0;
	struct partial_mode_info pm_info;

	pm_info.start_x = pm_info.end_x = pm_info.start_y = pm_info.end_y = 0;

	retval = sscanf(buf, "%u %u\n", &(pm_info.start_y), &(pm_info.end_y));

	if (retval != 2) {
		printk("MXCFB Invalid arguments to partial_window\n");
		return -EINVAL;
	}

	mxcfb_set_partial_mode(&pm_info);

	return (retval < 0) ? retval : count;
}

static DRIVER_ATTR(partial_window, S_IRUSR|S_IWUSR, NULL, store_partial_window);

static uint32_t panel_index;
static uint32_t tango_index;
static uint32_t ipu_index;

static ssize_t show_panel_read(struct device_driver *drv, char *buf)
{
	return sprintf(buf, "Rodem register 0x%X = 0x%X\n", panel_index, reg_read(panel_index));
}
static ssize_t store_panel_read(struct device_driver *drv, const char *buf, size_t count) 
{
	sscanf(buf, "%X", &panel_index);
	return count;
}
static DRIVER_ATTR(panel_read, S_IRUSR|S_IWUSR, show_panel_read, store_panel_read);

static ssize_t store_panel_write(struct device_driver *drv, const char *buf, size_t count) 
{
	uint32_t index, command;
	sscanf(buf, "%X %X", &index, &command);
	if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
		return -ERESTARTSYS;
	}
	reg_write(index, command);
	up(&mxcfb_global_state.g_sem);

	return count;
}
static DRIVER_ATTR(panel_write, S_IRUSR|S_IWUSR, NULL, store_panel_write);

static ssize_t show_ipu_read(struct device_driver *drv, char *buf)
{
	return sprintf(buf, "Rodem register 0x%lX = 0x%lX\n", ipu_index, 
			(*((volatile unsigned long *)(IO_ADDRESS(ipu_index)))));
}
static ssize_t store_ipu_read(struct device_driver *drv, const char *buf, size_t count) 
{
	sscanf(buf, "%X", &ipu_index);
	return count;
}
static DRIVER_ATTR(ipu_read, S_IRUSR|S_IWUSR, show_ipu_read, store_ipu_read);

static ssize_t store_ipu_write(struct device_driver *drv, const char *buf, size_t count) 
{
	uint32_t index, command;
	sscanf(buf, "%X %X", &index, &command);
	(*((volatile unsigned long *)(IO_ADDRESS(index)))) = command;

	return count;
}
static DRIVER_ATTR(ipu_write, S_IRUSR|S_IWUSR, NULL, store_ipu_write);


extern void tango_reg_write(uint32_t index, uint32_t command);
extern void tango_reg_read(uint32_t index);

static ssize_t store_tango_write(struct device_driver *drv, const char *buf, size_t count) 
{
	uint32_t index, command;
	sscanf(buf, "%X %X", &index, &command);

	if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
		return -ERESTARTSYS;
	}
	tango_reg_write(index, command);
	up(&mxcfb_global_state.g_sem);

	return count;
}
static DRIVER_ATTR(tango_write, S_IRUSR|S_IWUSR, NULL, store_tango_write);

static ssize_t show_tango_read(struct device_driver *drv, char *buf)
{	
	ssize_t retval;

	if(down_interruptible(&mxcfb_global_state.g_sem) != 0) {
		return -ERESTARTSYS;
	}

	reg_write(0x332, (tango_index << 8));
	reg_write(0x330, 0x6);
	msleep(20);
	retval = sprintf(buf, "Tango register 0x%X = 0x%X\n", tango_index, reg_read(0x33A));

	up(&mxcfb_global_state.g_sem);
	return retval;
}
static ssize_t store_tango_read(struct device_driver *drv, const char *buf, size_t count) 
{
	sscanf(buf, "%X", &tango_index);
	return count;
}
static DRIVER_ATTR(tango_read, S_IRUSR|S_IWUSR, show_tango_read, store_tango_read);

/*
 * Set up "brightness" as a driver attribute.
 */
static ssize_t show_brightness(struct device_driver *drv, char *buf)
{
	return sprintf(buf, "Brightness Level: %d\n", mxcfb_global_state.brightness); 
}

static ssize_t store_brightness(struct device_driver *drv, const char *buf, size_t count) 
{
	uint32_t brightness;
	int retval;

	if (sscanf(buf, "%u", &brightness) != 1) {
		return -EINVAL;
	}
	retval = mxcfb_ioctl(NULL, NULL, FBIOSETBRIGHTNESS, brightness, mxcfb_drv_data.fbi);

	return (retval < 0) ? retval : count;
}
static DRIVER_ATTR(brightness, S_IRUSR|S_IWUSR, show_brightness, store_brightness);

#endif /* MXCFB_DEBUG */

int __init mxcfb_setup(char *options)
{
        char *this_opt = NULL;

        if (!options || !*options)
                return -1;

	DPRINTK("mxcfb_setup: %s\n",options);

        while ((this_opt = strsep(&options, ",")) != NULL) {
                if (!strncmp(this_opt, "vram:", 5)) {
                        char *suffix;
                        def_vram = (simple_strtoul(this_opt + 5, &suffix, 0));
                        switch (suffix[0]) {
                        case 'm':
                        case 'M':
                                def_vram *= 1024 * 1024;
                                break;
                        case 'k':
                        case 'K':
                                def_vram *= 1024;
                                break;
                        default:
				;
			}
                }
                
	}
	return 0;
}

/*!
 * Main entry function for the framebuffer. The function registers the power
 * management callback functions with the kernel and also registers the MXCFB
 * callback functions with the core Linux framebuffer driver \b fbmem.c
 *
 * @return      Error code indicating success or failure
 */
int __init mxcfb_hvga_init(void)
{
	int ret = 0;
#ifndef MODULE
	char *option = NULL;
#endif
	FUNC_START;

        flipkeyd_task = kthread_create(flipkeyd, NULL, "flipkeyd");
        if (IS_ERR(flipkeyd_task)) {
                printk(KERN_ERR "mxcfb_hvga: flipkeyd creation failed!\n");
                return -ENOMEM;
        }

#ifndef MODULE
	if (fb_get_options("mxcfb", &option))
		return -ENODEV;
	mxcfb_setup(option);
#endif

        ret = driver_register(&mxcfb_driver);
#ifdef MXCFB_DEBUG
	driver_create_file(&mxcfb_driver, &driver_attr_panel_state);
	driver_create_file(&mxcfb_driver, &driver_attr_brightness);
	driver_create_file(&mxcfb_driver, &driver_attr_static_mode);
	driver_create_file(&mxcfb_driver, &driver_attr_swap_buffers);
	driver_create_file(&mxcfb_driver, &driver_attr_partial_window);
	driver_create_file(&mxcfb_driver, &driver_attr_spi_refresh);
	driver_create_file(&mxcfb_driver, &driver_attr_panel_read);
	driver_create_file(&mxcfb_driver, &driver_attr_panel_write);
	driver_create_file(&mxcfb_driver, &driver_attr_ipu_read);
	driver_create_file(&mxcfb_driver, &driver_attr_ipu_write);
	driver_create_file(&mxcfb_driver, &driver_attr_tango_read);
	driver_create_file(&mxcfb_driver, &driver_attr_tango_write);
#endif /* MXCFB_DEBUG */
        if (ret == 0) {
                ret = platform_device_register(&mxcfb_device);
                if (ret != 0) {
                        driver_unregister(&mxcfb_driver);
                }
        }

        FUNC_END;
        return ret;
}

void __exit mxcfb_hvga_exit(void)
{
        struct fb_info *fbi = mxcfb_drv_data.fbi;
        struct fb_info *fbi_ovl = mxcfb_drv_data.fbi_ovl;
        struct fb_info *fbi_cli = mxcfb_drv_data.fbi_cli;

        kthread_stop(flipkeyd_task);
        unregister_reboot_notifier(&powerdown_notifier);

        if (fbi) {
		mxcfb_blank(FB_BLANK_POWERDOWN, fbi);
                mxcfb_unmap_video_memory(fbi);

                if (fbi)
                        fb_dealloc_cmap(&fbi->cmap);

                unregister_framebuffer(fbi);
                framebuffer_release(fbi);
        }
        if (fbi_ovl) {
                mxcfb_unmap_video_memory(fbi_ovl);

                if (fbi_ovl)
                        fb_dealloc_cmap(&fbi_ovl->cmap);

                unregister_framebuffer(fbi_ovl);
                framebuffer_release(fbi_ovl);
        }
        if (fbi_cli) {
		mxcfb_blank(FB_BLANK_POWERDOWN, fbi_cli);
                mxcfb_unmap_video_memory(fbi_cli);

                if (fbi_cli)
                        fb_dealloc_cmap(&fbi_cli->cmap);

                unregister_framebuffer(fbi_cli);
                framebuffer_release(fbi_cli);
        }

#if defined(CONFIG_MOT_FEAT_LCD_ESD_RECOVERY)
	del_timer_sync(&esd_timer);
	flush_workqueue(esd_workqueue);
	destroy_workqueue(esd_workqueue);
#endif

        platform_device_unregister(&mxcfb_device);
        driver_unregister(&mxcfb_driver);
}

module_init(mxcfb_hvga_init);
module_exit(mxcfb_hvga_exit);

EXPORT_SYMBOL(mxcfb_set_static_mode);

MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MXC HVGA framebuffer driver");
MODULE_SUPPORTED_DEVICE("fb");
