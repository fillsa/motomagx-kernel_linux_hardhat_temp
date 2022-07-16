/*
 *  mxc_camera.c
 *
 *  Camera Interface driver.
 *
 *  Copyright (C) 2003, Intel Corporation
 *  Copyright (C) 2003, Montavista Software Inc.
 *  Copyright (C) 2003-2007 Motorola Inc.
 *
 *  Author: Intel Corporation Inc.
 *          MontaVista Software, Inc.
 *           source@mvista.com
 *          Motorola Inc.
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
                   Modification     
Author                 Date        Description of Changes
----------------   ------------    -------------------------
Motorola            12/19/2003     Created
Motorola            01/01/2004     Modified
Motorola            02/05/2004     Set frame rate in video mode
Motorola            02/26/2004     New chip id support
                                   Update algorithm for DMA transfer
                                   Update strategy for memory management
                                   Fix still picture capture failed sometime
                                   New Agilent sensor chip ID support
                                   Make output height in an even multiple of 8
                                   Dynamic power management feature 
Motorola            03/08/2004     Photo effects setting
                                   Fix segmentation fault in rmmod
                                   Adjust default image buffer size  
Motorola            04/26/2004     Power Management added
                                   Photo effects setting bug fix
Motorola            05/28/2004     add two new interface.
                                   1 get ready frames
                                   2 set frame buffer count
Motorola            06/22/2005     Make changes to build with
                                   2.6 kernel for SCMA11
Motorola            08/09/2005     Make use of IPU on SCM-A11
Motorola            08/25/2005     Adopt new IPU API
Motorola            09/08/2005     Use new power API
Motorola            09/09/2005     Upmerge for new MontaVista release
Motorola            10/03/2005     Redefine the purpose of page_array
                                   and use it to directly store
                                   physical addresses used by IPU
Motorola            10/04/2005     Make timing changes associated
                                   with viewfinder hang:
                                   a. correct master clock
                                   b. correct use of IPU API
                                   c. insert delay allowing VCAM to
                                      settle when toggling it.
Motorola            11/16/2005     Upmerge to EZXBASE 36
Motorola            12/09/2005     Fix EZXBASE36 upmerge issues
Motorola            01/04/2006     Update fxn prototype due to
                                   MontaVista 12/19/2005 upmerge
Motorola            01/27/2006     Add support for Ascension flash
Motorola            02/15/2006     Add support for Omnivision
                                   OV2640 (viewfinder only)
Motorola            03/03/2006     EZXBASE 48 upmerge
Motorola            03/08/2006     Add support for dynamic switching 
                                   of cameras
Motorola            03/15/2006     Add support for Sensor 2M
Motorola            03/15/2006     Support fullsize capture
Motorola            06/02/2006     Fix clock setting for Sensor 2M A 
                                   and Sensor 2M
Motorola            06/09/2006     Upmerge to MV 05/12/2006
Motorola            07/12/2006     Viewfinder causing very long response
                                   delay to user input
Motorola            08/07/2006     Enable workarounds during capture
Motorola            09/05/2006     the mdelay() in camera driver will
                                   decrease system's performance
Motorola            10/11/2006     Increase camera mclk to 26.6MHz for Sensor 2M
Motorola            11/01/2006     Mediaplayer crashes when launch camera
Motorola            11/07/2006     Handle IPU CSI data overflow condition
Motorola            11/30/2006     Enable flash, if enabled,  prior to capture
Motorola            12/07/2006     OSS code changes
Motorola            12/14/2006     Prevent flash from pulsating
Motorola            01/04/2007     OSS code changes
Motorola            01/17/2007     Improve zoom quality
Motorola            02/20/2007     Desense improvements

*/


/*================================================================================
                                 INCLUDE FILES
================================================================================*/  
#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/power_ic_kernel.h>

#include <linux/types.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/irq.h>

#include "mxc_camera.h"
#include "../drivers/mxc/ipu/ipu.h"
#include "../drivers/mxc/ipu/ipu_regs.h"
#include "mxc/capture/ipu_prp_sw.h"
#include "mxc/capture/mxc_v4l2_capture.h"

#ifdef CONFIG_MOT_FEAT_2MP_CAMERA_WRKARND
#include "mxc/capture/capture_2mp_wrkarnd.h"
extern void ipu_sdc_fg_uninit(void);
#endif

#ifdef CONFIG_MACH_ASCENSION
#include <linux/lights_funlights.h>
#endif

#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
#include <linux/fb.h>
#define FB_DEV_INDEX     0
#define FB_OVL_DEV_INDEX 1
extern struct fb_info *registered_fb[FB_MAX];
#endif /* CONFIG_MXC_IPU_PRP_VF_SDC */

extern void gpio_sensor_active(void);
extern void gpio_sensor_inactive(void);
extern int prp_enc_setup(cam_data *cam);

/*
 * It is required to have at least 3 frames in buffer
 * in current implementation
 */
#define FRAMES_IN_BUFFER    	3

#define MAX_PIXELS_PER_LINE     2047   /* totally 11 bits */
#define MAX_LINES_PER_FRAME     2047   /* totally 11 bits */
static camera_context_t  *g_camera_context = NULL;
  #ifdef CONFIG_MXC_IPU_CAMERA_SENSOR2M
  extern camera_function_t  sensor2m_func;
  extern int camera_func_sensor2m_s9(void);
  #endif
wait_queue_head_t  camera_wait_q;	

/* /dev/videoX registration number */
static int    minor     = 0;
static int    last_error = CAMERA_ERROR_NONE;

#undef LOG_TIME_STAMP   //If defined, the time stamp log will be printed out

#ifdef LOG_TIME_STAMP
#define MAX_TIME_LOG 5
struct timeval tv0,tv1,tv2[MAX_TIME_LOG],tv3[MAX_TIME_LOG],tv4, tv5, tv6;
int    first_frame = 0;
int    time_log_num = 0;
#endif

#define CAPTURE_ST_IDLE             0
#define CAPTURE_ST_SETUP_SKIPFRAME  1
#define CAPTURE_ST_WAIT_RISINGFV    2
#define CAPTURE_ST_WAIT_FALLINGFV   3
#define CAPTURE_ST_WAIT_SOF         4
#define CAPTURE_ST_WAIT_DATADONE    5
#define CAPTURE_ST_DATADONE         6

int camera_deinit(p_camera_context_t camera_context);

static void start_dma_transfer(p_camera_context_t camera_context, unsigned block_id);
static void stop_dma_transfer(p_camera_context_t camera_context);
static int  start_capture(p_camera_context_t camera_context, unsigned int block_id, unsigned int frames);
static void start_capture_data(p_camera_context_t camera_context);

static int mxc_camera_open(struct inode *inode, struct file *file);
static int mxc_camera_close(struct inode *inode, struct file *file);
static ssize_t mxc_camera_read(struct file *file, char *buf, size_t count, loff_t *ppos);
static unsigned int mxc_camera_poll(struct file *file, poll_table *wait);
static int mxc_camera_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int mxc_camera_mmap(struct file *file, struct vm_area_struct *vma);

static int mxc_camera_mem_deinit(void);
static int mxc_camera_mem_init(void);

static void ci_set_image_format(p_camera_context_t camera_context, int width, int height, int format); 

static int deinterleave(p_camera_context_t cam_ctx);
static void camera_sysfs_release(struct video_device *vfd);

static struct file_operations mxc_fops = {
  .owner   = THIS_MODULE,
  .open	   = mxc_camera_open,
  .release = mxc_camera_close,
  .read	   = mxc_camera_read,
  .poll	   = mxc_camera_poll,
  .ioctl   = mxc_camera_ioctl,
  .mmap	   = mxc_camera_mmap,
};

static struct video_device vd = {
  .owner   = THIS_MODULE,
  .name    = "camera",
  .type	   = VID_TYPE_CAPTURE,
  .fops    = &mxc_fops,
  .release = camera_sysfs_release,
};

/* FY: defined for vf task, used in ipu_prp_vf_sdc.c as well */
V4l_VF_PARAM vfparam;

static cam_data ipu_cam_api;
/* NOTE: output buffer mmap'ed to application */
static unsigned char *buf_addr_physical;
static unsigned char *buf_addr_virtual;
static ipu_csi_signal_cfg_t param;

static uint32_t p_mclk_freq = 27000000; /* NOTE: request 26MHz clock */
                                 /* NOTE: add 1MHz to allow IPU
                                    to supply 26.6MHz; otherwise, IPU
                                    will go to next lower divisor to
                                    generate 22.1MHz clock */

#ifdef CONFIG_MACH_ASCENSION
/* NOTE: variable tells the camera driver whether to 
         turn on the flash when taking snapshot.  Its
         value is set by the user-space app via ioctl(). */
static int flash;
static unsigned long flash_last_on_time;
#endif

/* NOTE: table and utility fxn for converting the format enum
         to V4L2 format enum */
static int palette2pixelformat[] = {
	[CAMERA_IMAGE_FORMAT_RGB555]             = V4L2_PIX_FMT_RGB555,
	[CAMERA_IMAGE_FORMAT_RGB565]             = V4L2_PIX_FMT_RGB565,
	[CAMERA_IMAGE_FORMAT_RGB888_PACKED]      = V4L2_PIX_FMT_BGR24,
	[CAMERA_IMAGE_FORMAT_RGBT888_0]          = V4L2_PIX_FMT_BGR32,
	/* yuv packed pixel */
	[CAMERA_IMAGE_FORMAT_YCBCR422_PACKED]    = V4L2_PIX_FMT_YUYV,
	/* yuv planar */
	[CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR]    = V4L2_PIX_FMT_YUV422P,
};

static unsigned int
palette_to_pixelformat(unsigned int palette)
{
	if (palette < ARRAY_SIZE(palette2pixelformat))
		return palette2pixelformat[palette];
	else
		return 0;
}


/***********************************************************************
 *
 * Camera lock functions
 *
 ***********************************************************************/

static struct semaphore camera_lock_sem;

static void camera_lock_init(void)
{
    init_MUTEX(&camera_lock_sem);
}

void camera_lock(void)
{
    ddbg_print("lock");
    down(&camera_lock_sem);
    ddbg_print("locked");
}

int camera_trylock(void)
{
    int ret;
    ddbg_print("trylock");
    ret = down_trylock(&camera_lock_sem);
    if(ret)
    {
        ddbg_print("lock fail");
        return -1;
    }
    ddbg_print("locked");
    return 0;
}

void camera_unlock(void)
{
    ddbg_print("unlock");
    up(&camera_lock_sem);
    ddbg_print("unlocked");
}


#ifdef LOG_TIME_STAMP
static char buf[256];
static void camera_capture_time_log(int capture)
{
    int i;
    struct timeval tv_temp;
    char buf1[32];
    int t, t1;
    printk("B: %ld, %ld E: %ld, %ld T: %ld\n", (long)tv0.tv_sec, tv0.tv_usec/1000, 
                                             (long)tv4.tv_sec, tv4.tv_usec/1000,
                                             (tv4.tv_sec-tv0.tv_sec)*1000+ (tv4.tv_usec-tv0.tv_usec)/1000);
  
    sprintf(buf, "WR: %ld ", (tv1.tv_sec-tv0.tv_sec)*1000+ (tv1.tv_usec-tv0.tv_usec)/1000);
    
    for(i = 0; i < time_log_num; i++)
    {
        if(i == 0)
            tv_temp = tv1;
        else
            tv_temp = tv3[i - 1];
        sprintf(buf1, "%d> FVR:%ld FVF:%ld ", i+1, (tv2[i].tv_sec-tv_temp.tv_sec)*1000+ (tv2[i].tv_usec-tv_temp.tv_usec)/1000, 
                                               (tv3[i].tv_sec-tv2[i].tv_sec)*1000+ (tv3[i].tv_usec-tv2[i].tv_usec)/1000);
        strcat(buf, buf1);
    }
    sprintf(buf1, "L: %ld ", (tv4.tv_sec-tv3[time_log_num - 1].tv_sec)*1000+ (tv4.tv_usec-tv3[time_log_num - 1].tv_usec)/1000);
    strcat(buf, buf1);
    if(capture)
    {
        t = (tv5.tv_sec-tv4.tv_sec)*1000+ (tv5.tv_usec-tv4.tv_usec)/1000;
        if(t == 0)
        {
            strcat(buf, "us");
            t = (tv5.tv_usec-tv4.tv_usec);  
        }  
        t1 = (tv6.tv_sec-tv5.tv_sec)*1000+ (tv6.tv_usec-tv5.tv_usec)/1000;
        if(t1 == 0)
            t1 = (tv6.tv_usec-tv5.tv_usec);
        sprintf(buf1, "SOF: %d Data: %d\n", t, t1);    
        strcat(buf, buf1);
    }
    else
        strcat(buf, "\n");
    printk(buf);

}
#endif

/***********************************************************************
 *
 * Private functions
 *
 ***********************************************************************/

/* NOTE: this fxn is analogous to mxc_allocate_frame_buf() in mxc_v4l2_capture.c */ 
static int mxc_dma_buffer_init(p_camera_context_t camera_context)
{
	unsigned int	pages;
	unsigned int	page_count;

	camera_context->pages_allocated = 0;

	pages = (PAGE_ALIGN(camera_context->buf_size) / PAGE_SIZE);

	camera_context->page_array = (struct page **)
                                 kmalloc(pages * sizeof(struct page *),
                                 GFP_KERNEL);
                               
	if(camera_context->page_array == NULL)
	{
                err_print("alloc memory for page_array fail, %d bytes", pages*4);
		return -ENOMEM;
	}
	memset(camera_context->page_array, 0, pages * sizeof(struct page *));

	/* NOTE: get physical address of camera buffer */
	buf_addr_physical = (unsigned char *)ipu_malloc(PAGE_ALIGN(camera_context->buf_size));
	if(buf_addr_physical == NULL)
	  {
	    printk("mxc_dma_buffer_init IPU_Malloc failed.\n");
	    goto error;
	  }

	/* NOTE: fill up page_array with physical addresses to be used by IPU */
	for(page_count = 0; page_count < pages; page_count++)
	{
		camera_context->page_array[page_count] = (struct page *)
		  (buf_addr_physical+page_count*PAGE_SIZE);
	}

	camera_context->pages_allocated = pages;

	return 0;

error:
	if(buf_addr_physical != NULL)
	  {
	    ipu_free((unsigned long)buf_addr_physical);
	    buf_addr_physical = NULL;
	  }

	kfree(camera_context->page_array);
	camera_context->page_array = NULL;

	return -ENOMEM;
}

static void mxc_dma_buffer_free(p_camera_context_t camera_context)
{
  if(buf_addr_physical == NULL)
    return;

	ipu_free((unsigned long)buf_addr_physical);
	buf_addr_physical = NULL;

	kfree(camera_context->page_array);
	camera_context->page_array = NULL;
}

/* NOTE: this fxn is analogous to mxc_streamon() in mxc_v4l2_capture.c */
static void start_dma_transfer(p_camera_context_t camera_context, unsigned block_id)
{
#ifdef CONFIG_MACH_ASCENSION
  unsigned long current_time;
  unsigned long flash_cool_off_time;
#endif

  if(camera_context->still_image_mode == 0)
    {
      /* NOTE: video mode */

      /* NOTE: setup and enable DMA channel(s) */
      if (ipu_cam_api.enc_enable)
	ipu_cam_api.enc_enable(&ipu_cam_api);

      /* NOTE: setup buffer and trigger DMA transfer */
      ipu_cam_api.ping_pong_csi = 0;
      if (ipu_cam_api.enc_update_eba)
	ipu_cam_api.enc_update_eba((u32)
	  (camera_context->page_array[block_id * camera_context->pages_per_block]), 
          &ipu_cam_api.ping_pong_csi);
#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
      if(camera_context->vf_configured) {
        if(ipu_cam_api.vf_start_sdc) {
          if(ipu_cam_api.vf_start_sdc(&ipu_cam_api) != 0)
            ddbg_print("Failed to start VF");
        }
      }
#endif /* CONFIG_MXC_IPU_PRP_VF_SDC */
    }
  else
    {
      /* NOTE: still-image mode */
#ifdef CONFIG_MACH_ASCENSION
    /* NOTE: enable flash if needed in still image mode */
    if(flash)
      {
	current_time = jiffies;
	if(current_time > flash_last_on_time)
	  {
	    flash_cool_off_time = current_time - flash_last_on_time;
	  }
	else
	  {
	    /* adjust for wrap around */
	    flash_cool_off_time = (0UL - 1) - flash_last_on_time + current_time;
	  }

	if(flash_cool_off_time < (HZ*12/10))
	  {
	    msleep(1200 - flash_cool_off_time * 1000 / HZ);
	  }

	/* NOTE: reset time-out protection just in case */
	lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT, 1, LIGHTS_FL_REGION_CAMERA_FLASH, LIGHTS_FL_COLOR_BLACK);

	/* NOTE: enable flash */
	lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT, 1, LIGHTS_FL_REGION_CAMERA_FLASH, LIGHTS_FL_CAMERA_FLASH);
      }
#endif

      ipu_cam_api.still_buf = (void *)
	(camera_context->page_array[block_id * camera_context->pages_per_block]);

      /* Clear out CSI_EOF bit to prevent false eof interrupt */
      __raw_writel(0x00000040L, IPU_INT_STAT_3);
      ipu_cam_api.csi_start(&ipu_cam_api);

#ifdef CONFIG_MOT_FEAT_2MP_CAMERA_WRKARND
      /* Disable Foreground channel*/
      ipu_sdc_fg_uninit();
#endif
    }

  camera_context->dma_started = 1;

}


/* NOTE: this fxn is analogous to mxc_streamoff() in mxc_v4l2_capture.c */
static void stop_dma_transfer(p_camera_context_t camera_context)
{
  if(camera_context->dma_started == 1)
    {
      if(camera_context->still_image_mode == 0)
	{
	  /* NOTE: video mode */
	  /* NOTE: gracefully stop camera data transfer */
	  if (ipu_cam_api.enc_disable)
	    {
	      ipu_cam_api.enc_disable(&ipu_cam_api);
	      ipu_disable_channel(CSI_PRP_ENC_MEM, false);
	    }

#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
      if(ipu_cam_api.overlay_active) {
        if(ipu_cam_api.vf_stop_sdc) {
          if(ipu_cam_api.vf_stop_sdc(&ipu_cam_api) != 0)
            ddbg_print("Failed to stop VF");
          }
      }
#endif /* CONFIG_MXC_IPU_PRP_VF_SDC */
	}
      else
	{
	  /* NOTE: still-image mode */
	  ipu_cam_api.csi_stop(&ipu_cam_api);
	}

      camera_context->dma_started = 0;
    }
}

static int start_capture(p_camera_context_t camera_context, unsigned int block_id, unsigned int frames)
{
    int status;
    status = camera_context->camera_functions->start_capture(camera_context, frames);

	return status;
}
 
/***********************************************************************
 *
 * Init/Deinit APIs
 *
 ***********************************************************************/
int camera_sensor_init(p_camera_context_t camera_context)
{
    int ret=0;
    ddbg_print(""); 
  
    camera_lock();

    // init context status, other member is set to 0
    camera_context->sensor_width  = 640;
    camera_context->sensor_height = 480;

    camera_context->capture_width  = 320;
    camera_context->capture_height = 240;

    camera_context->still_input_format  = CAMERA_IMAGE_FORMAT_YCBCR422_PACKED;
    camera_context->still_output_format = CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;

    camera_context->capture_input_format  = CAMERA_IMAGE_FORMAT_YCBCR422_PACKED;
    camera_context->capture_output_format = CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;

    camera_context->capture_digital_zoom = 256; // 1x zoom
    camera_context->still_digital_zoom = 256;   // 1x zoom

    camera_context->plane_number = 1;

    camera_context->still_image_mode = 0;
    camera_context->still_image_rdy = 0;

    camera_context->task_waiting = 0;
    camera_context->detected_sensor_type = 0;

    camera_context->preferred_block_num = FRAMES_IN_BUFFER;

    camera_context->capture_style = V4l_STYLE_NORMAL;
    camera_context->capture_light = V4l_WB_AUTO;
    camera_context->capture_bright = 0;
    camera_context->flicker_freq = 50;

    camera_context->buf_size     = 0;
    camera_context->dma_descriptors_size = (camera_context->buf_size/PAGE_SIZE + 10);

    camera_context->capability.max_width  = MAX_PIXELS_PER_LINE;
    camera_context->capability.max_height = MAX_LINES_PER_FRAME;
    camera_context->capability.min_width  = 64;
    camera_context->capability.min_height = 48;

    /*init sensor */
    if(camera_context->detected_sensor_type == 0) {
    /* test sensor type */

  #ifdef CONFIG_MXC_IPU_CAMERA_SENSOR2M
    dbg_print("detect SENSOR2M...");
    camera_context->camera_functions = &sensor2m_func;
    if((ret = camera_context->camera_functions->init(camera_context)) ==0)
    {
	    // Reinitialize CSI for SENSOR2M to use gated clock mode
	    param.clk_mode   = 0x0; // gated clock mode
	    param.pixclk_pol = 0x0; // no pixel clock inversion
	    param.ext_vsync  = 0x1; // use external vsync
	    p_mclk_freq = 27000000; // request 27MHz, will get 26.6MHz
	    set_mclk_rate(&p_mclk_freq);
	    ipu_csi_init_interface(200 -1, 160 -1, IPU_PIX_FMT_UYVY, param);
	    ipu_csi_set_window_size(200, 160);
	    /* Master clock has just been re-enabled; allow settling time */
	    mdelay(1);
	    goto test_success;
    }
  #endif

        camera_context->camera_functions = 0;
	    
        err_print("camera function init error!!");
        goto camera_sensor_init_err;
    }
    else 
    {
        switch (camera_context->detected_sensor_type) {
    #ifdef CONFIG_MXC_IPU_CAMERA_SENSOR2M
                case CAMERA_TYPE_SENSOR2M:
                    camera_context->camera_functions = &sensor2m_func;
                break;
    #endif
        }
        camera_context->camera_functions->init(camera_context);
    }
test_success:
    camera_context->detected_sensor_type = camera_context->sensor_type;

    return 0;

camera_sensor_init_err:
    camera_deinit(camera_context);
    return -ENODEV; 
}

void camera_gpio_init(void)
{
    /* NOTE: configure GPIO */
    gpio_sensor_active();
}   

int camera_deinit( p_camera_context_t camera_context )
{
    // deinit sensor
	if(camera_context->camera_functions)
		camera_context->camera_functions->deinit(camera_context);  
	
	// capture interface deinit
	ci_deinit();

        camera_unlock();
	return 0;
}

int camera_ring_buf_init(p_camera_context_t camera_context)
{
    int i;
	unsigned         frame_size;
    unsigned int width, height;
    unsigned int output_format;
    dbg_print("");    
    if(camera_context->still_image_mode)
    {
        width = camera_context->still_width;
        height = camera_context->still_height;
        output_format = camera_context->still_output_format;
    }
    else
    {
         width = camera_context->capture_width;
         height = camera_context->capture_height;
         output_format = camera_context->capture_output_format;
    }
    
    switch(output_format)
    {
    case CAMERA_IMAGE_FORMAT_RGB565:
        frame_size = width * height * 2;
        camera_context->fifo0_transfer_size = frame_size;
        camera_context->fifo1_transfer_size = 0;
        camera_context->fifo2_transfer_size = 0;
        camera_context->plane_number = 1;
        break;
    case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
        frame_size = width * height * 2;
        camera_context->fifo0_transfer_size = frame_size;
        camera_context->fifo1_transfer_size = 0;
        camera_context->fifo2_transfer_size = 0;
        camera_context->plane_number = 1;
        break;
    case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
        frame_size = width * height * 2;
        camera_context->fifo0_transfer_size = frame_size / 2;
        camera_context->fifo1_transfer_size = frame_size / 4;
        camera_context->fifo2_transfer_size = frame_size / 4;
        camera_context->plane_number = 3;
        break;
    case CAMERA_IMAGE_FORMAT_RGB666_PLANAR:
        frame_size = width * height * 4;
        camera_context->fifo0_transfer_size = frame_size;
        camera_context->fifo1_transfer_size = 0;
        camera_context->fifo2_transfer_size = 0;
        camera_context->plane_number = 1;
        break;
    case CAMERA_IMAGE_FORMAT_RGB666_PACKED:
        frame_size = width * height * 3;
        camera_context->fifo0_transfer_size = frame_size;
        camera_context->fifo1_transfer_size = 0;
        camera_context->fifo2_transfer_size = 0;
        camera_context->plane_number = 1;
        break;
    case CAMERA_IMAGE_FORMAT_RAW8:
        frame_size = width * height * 1;
        camera_context->fifo0_transfer_size = frame_size;
        camera_context->fifo1_transfer_size = 0;
        camera_context->fifo2_transfer_size = 0;
        camera_context->plane_number = 1;
        break;
    default:
        return -EINVAL;
        break;
    }

    camera_context->block_size = frame_size;

	camera_context->pages_per_fifo0 =
		(PAGE_ALIGN(camera_context->fifo0_transfer_size) / PAGE_SIZE);
	camera_context->pages_per_fifo1 =
		(PAGE_ALIGN(camera_context->fifo1_transfer_size) / PAGE_SIZE);
	camera_context->pages_per_fifo2 =
		(PAGE_ALIGN(camera_context->fifo2_transfer_size) / PAGE_SIZE);

	camera_context->pages_per_block =
		camera_context->pages_per_fifo0 +
		camera_context->pages_per_fifo1 +
		camera_context->pages_per_fifo2;

	camera_context->page_aligned_block_size =
		camera_context->pages_per_block * PAGE_SIZE;

	camera_context->block_number_max =
		camera_context->pages_allocated /
		camera_context->pages_per_block;


    //restrict max block number
    if(camera_context->block_number_max > MAX_FRAMES)
    {
       camera_context->block_number_max = MAX_FRAMES;
    }
 
    if(camera_context->block_number_max > FRAMES_IN_BUFFER)
    {
       camera_context->block_number = camera_context->preferred_block_num;//FRAMES_IN_BUFFER; 
    }
    else
    {
       camera_context->block_number = camera_context->block_number_max;
    }

    if((camera_context->still_image_mode && camera_context->block_number < 1) 
         || (!camera_context->still_image_mode && camera_context->block_number < FRAMES_IN_BUFFER))
    {
        err_print("Out of Memory");
        return -ENOMEM;
    }

    for(i=0; i<camera_context->block_number; i++) {
        camera_context->planeBytes[i][0] = camera_context->fifo0_transfer_size;
        camera_context->planeOffset[i][0] = i*camera_context->page_aligned_block_size;
        camera_context->planeBytes[i][1] = camera_context->fifo1_transfer_size;
        camera_context->planeOffset[i][1] = camera_context->planeOffset[i][0] 
                                          + camera_context->pages_per_fifo0*PAGE_SIZE;
        camera_context->planeBytes[i][2] = camera_context->fifo2_transfer_size;
        camera_context->planeOffset[i][2] = camera_context->planeOffset[i][1] 
                                          + camera_context->pages_per_fifo1*PAGE_SIZE;
    }

	camera_context->block_header = camera_context->block_tail = 0;
	return 0;

}
/***********************************************************************
 *
 * Capture APIs
 *
 ***********************************************************************/
// Set the image format
int camera_set_capture_format(p_camera_context_t camera_context, int set_sensor)
{

	//int status;
    unsigned int width, height;

    unsigned int input_format, output_format;

    if(camera_context == NULL || camera_context->camera_functions == NULL ||
       camera_context->camera_functions->set_capture_format == NULL)
    {
      err_print("camera_context point NULL!!!");
      return -EFAULT;
    }


    if(camera_context->capture_input_format >  CAMERA_IMAGE_FORMAT_MAX ||
       camera_context->capture_output_format > CAMERA_IMAGE_FORMAT_MAX ||
       camera_context->still_input_format >  CAMERA_IMAGE_FORMAT_MAX   ||
       camera_context->still_output_format >  CAMERA_IMAGE_FORMAT_MAX)
    {
        err_print("format error");
        return -EINVAL;
    }

    // set sensor setting, it will also setup some context parameter
    if(set_sensor)
    {
       if (camera_context->camera_functions->set_capture_format(camera_context))
       {
           err_print("sensor set_capture_format failed");
           return -EIO;
       }
    }

    if(camera_context->still_image_mode)
    {
        input_format  = camera_context->still_input_format;
        output_format = camera_context->still_output_format;
        width = camera_context->still_width;
        height = camera_context->still_height;
    }
    else
    {
        input_format  = camera_context->capture_input_format;
        output_format = camera_context->capture_output_format;
        width = camera_context->capture_width;
        height = camera_context->capture_height;
    }
    
    ddbg_print("w=%d h=%d, in format %d, out format %d", width, height,
            input_format, output_format);
    
    ci_set_image_format(camera_context, width, height, output_format);
 
    // ring buffer init
    return camera_ring_buf_init(camera_context);
    
}

// take a picture and copy it into the ring buffer
int camera_capture_still_image(p_camera_context_t camera_context, unsigned int block_id)
{
    int result;
    camera_context->still_image_mode = 1;
    camera_context->task_waiting = 0;
    camera_context->still_image_rdy = 0;
    result = camera_set_capture_format(camera_context, 0);
    if(result != 0)
    {
         err_print("camera_set_capture_format return error");
         return result;
    }
    // init buffer status & capture
    camera_context->block_header   = camera_context->block_tail = block_id;
    camera_context->capture_status = CAPTURE_ST_IDLE;
#ifdef LOG_TIME_STAMP
    first_frame = 1;
    do_gettimeofday(&tv0);
#endif
	return  start_capture(camera_context, block_id, 1);
    
}

// capture motion video and copy it to the ring buffer
int camera_start_video_capture( p_camera_context_t camera_context, unsigned int block_id, int set_sensor )
{
	//init buffer status & capture
    int result;
    camera_context->still_image_mode = 0;

    result = camera_set_capture_format(camera_context, set_sensor);
    if(result != 0)
        return result;
    //init buffer status & capture
    camera_context->block_header   = camera_context->block_tail = block_id;
    camera_context->capture_status = CAMERA_STATUS_VIDEO_CAPTURE_IN_PROCESS;
#ifdef LOG_TIME_STAMP
    first_frame = 1;
    do_gettimeofday(&tv0);
#endif
	return start_capture(camera_context, block_id, 0);
}

// disable motion video image capture
void camera_stop_video_capture( p_camera_context_t camera_context )
{
	//stop dma
	stop_dma_transfer(camera_context);

	//stop capture
	camera_context->camera_functions->stop_capture(camera_context);
    
	//update the flag
	if(!(camera_context->capture_status & CAMERA_STATUS_RING_BUFFER_FULL))
    {
		camera_context->capture_status &= ~CAMERA_STATUS_VIDEO_CAPTURE_IN_PROCESS;
    }
}

//reset CI and start dma transfer
static void start_capture_data( p_camera_context_t camera_context)
{
#ifdef LOG_TIME_STAMP
    do_gettimeofday(&tv4);
#endif

    start_dma_transfer(camera_context, 0);

#ifdef LOG_TIME_STAMP
          do_gettimeofday(&tv5);
#endif
          camera_context->capture_status = CAPTURE_ST_WAIT_DATADONE;
     }

// skip frame before capture video or still image
void camera_skip_frame( p_camera_context_t camera_context, int waiting_frame )
{
    /* set the FV to GPIO mode to capture the raising and falling edge of FV */

    camera_context->capture_status = CAPTURE_ST_SETUP_SKIPFRAME;

    camera_context->waiting_frame = 0;
    start_capture_data(camera_context);
}

/***********************************************************************
 *
 * Flow Control APIs
 * 
 ***********************************************************************/
// continue capture image to next available buffer
void camera_continue_transfer( p_camera_context_t camera_context )
{
	// don't think we need this either.  JR
	// continue transfer on next block
	start_dma_transfer( camera_context, camera_context->block_tail );
}

// Return 1: there is available buffer, 0: buffer is full
int camera_next_buffer_available( p_camera_context_t camera_context )
{
	camera_context->block_header = (camera_context->block_header + 1) % camera_context->block_number;
	if(((camera_context->block_header + 1) % camera_context->block_number) != camera_context->block_tail)
	{
		return 1;
	}

	camera_context->capture_status |= CAMERA_STATUS_RING_BUFFER_FULL;
	return 0;
}

// Application supplies the FrameBufferID to the driver to tell it that the application has completed processing of 
// the given frame buffer, and that buffer is now available for re-use.
void camera_release_frame_buffer(p_camera_context_t camera_context, unsigned int frame_buffer_id)
{

	camera_context->block_tail = (camera_context->block_tail + 1) % camera_context->block_number;

	// restart video capture only ifvideo capture is in progress and space is available for image capture
	if((camera_context->capture_status & CAMERA_STATUS_RING_BUFFER_FULL ) && 
	   (camera_context->capture_status & CAMERA_STATUS_VIDEO_CAPTURE_IN_PROCESS))
	{
		if(((camera_context->block_header + 2) % camera_context->block_number) != camera_context->block_tail)
		{
			camera_context->capture_status &= ~CAMERA_STATUS_RING_BUFFER_FULL;
			start_capture(camera_context, camera_context->block_tail, 0);
		}
	}
}

// Returns the FrameBufferID for the first filled frame
// Note: -1 represents buffer empty
int camera_get_first_frame_buffer_id(p_camera_context_t camera_context)
{
	// not sure ifthis routine makes any sense.. JR

	// check whether buffer is empty
	if((camera_context->block_header == camera_context->block_tail) && 
		 !(camera_context->capture_status & CAMERA_STATUS_RING_BUFFER_FULL))
    {
	    return -1;
    }

	// return the block header
	return camera_context->block_header;
}

// Returns the FrameBufferID for the last filled frame, this would be used ifwe were polling for image completion data, 
// or we wanted to make sure there were no frames waiting for us to process.
// Note: -1 represents buffer empty
int camera_get_last_frame_buffer_id(p_camera_context_t camera_context)
{

	// check whether buffer is empty
	if((camera_context->block_header == camera_context->block_tail) && 
	     !(camera_context->capture_status & CAMERA_STATUS_RING_BUFFER_FULL))
    {
		return -1;
    }

	// return the block before the block_tail
	return (camera_context->block_tail + camera_context->block_number - 1) % camera_context->block_number;
}


/***********************************************************************
 *
 * Buffer Info APIs
 *
 ***********************************************************************/
// Return: the number of frame buffers allocated for use.
unsigned int camera_get_num_frame_buffers(p_camera_context_t camera_context)
{
	return camera_context->block_number;
}

// FrameBufferID is a number between 0 and N-1, where N is the total number of frame buffers in use.  Returns the address of
// the given frame buffer.  The application will call this once for each frame buffer at application initialization only.
void * camera_get_frame_buffer_addr(p_camera_context_t camera_context, unsigned int frame_buffer_id)
{
	return (void*)((unsigned)camera_context->buffer_virtual +
		 camera_context->page_aligned_block_size * frame_buffer_id);
}

// Return the block id
int camera_get_frame_buffer_id(p_camera_context_t camera_context, void* address)
{
	if(((unsigned)address >= (unsigned)camera_context->buffer_virtual) && 
	   ((unsigned)address <= (unsigned)camera_context->buffer_virtual + camera_context->buf_size))
    {
		return ((unsigned)address - 
                (unsigned)camera_context->buffer_virtual) / 
                camera_context->page_aligned_block_size;
    }

	return -1;
}


/***********************************************************************
 *
 * Interrupt APIs
 *
 ***********************************************************************/
void camera_turn_on_vcam(void)
{
  /* Enable power to camera */
    power_ic_periph_set_camera_on(POWER_IC_PERIPH_ON);
  /* VCAM rise-time measured to be about 100us */
  /* insert 200us delay to be safe */
    udelay(200);
}

void camera_turn_off_vcam(void)
{
  /* Disable power to camera */
  power_ic_periph_set_camera_on(POWER_IC_PERIPH_OFF);
  /* VCAM fall-time measured to be about 15ms */
  /* insert 20ms-delay to be safe */
  msleep(20);
}

/***********************************************************************************
* Application interface 							   *
***********************************************************************************/
static int mxc_camera_open(struct inode *inode, struct file *file)
{
    camera_context_t *cam_ctx;
    int ret;
    dbg_print("start...");

    /* allow one instance only */
    if((ret=video_exclusive_open(inode, file)))
      {
	return ret;
      }
 
    // Enable power to camera
    camera_turn_on_vcam();

    /* Allow settling time after enabling master clock */
    mdelay(1);

    /* NOTE: add camera related activation here and
             add camera related deactivation before returning with error */
    memset(&ipu_cam_api, 0, sizeof(cam_data));
    prp_enc_select(&ipu_cam_api);

    prp_still_select(&ipu_cam_api);
#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
    prp_vf_sdc_select(&ipu_cam_api);
#endif
#ifdef CONFIG_MXC_IPU_PRP_VF_ADC
    prp_vf_adc_select(&ipu_cam_api);
#endif
    init_waitqueue_head(&camera_wait_q);
    init_waitqueue_head(&ipu_cam_api.still_queue);

    /*alloc memory for camera context*/
    if(mxc_camera_mem_init())
    {
      prp_enc_deselect(&ipu_cam_api);
      prp_still_deselect(&ipu_cam_api);
#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
      prp_vf_sdc_deselect(&ipu_cam_api);
#endif
#ifdef CONFIG_MXC_IPU_PRP_VF_ADC
      prp_vf_adc_deselect(&ipu_cam_api);
#endif
      err_print("memory allocate failed!");
      camera_turn_off_vcam();
      video_exclusive_release(inode, file);
      return -ENOMEM;
    }

    cam_ctx = g_camera_context;
    
    /*
     camera_sensor_init call init function
    */
    if(camera_sensor_init(cam_ctx))
    {
        err_print("camera_sensor_init failed!");
        mxc_camera_mem_deinit();
	prp_enc_deselect(&ipu_cam_api);
	prp_still_deselect(&ipu_cam_api);
#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
        prp_vf_sdc_deselect(&ipu_cam_api);
#endif
#ifdef CONFIG_MXC_IPU_PRP_VF_ADC
        prp_vf_adc_deselect(&ipu_cam_api);
#endif
	camera_turn_off_vcam();
	video_exclusive_release(inode, file);
        return -EIO;
    }
    
    cam_ctx->dma_started = 0;

    /*
      alloc memory for picture buffer 
      init function of each sensor should set proper value for cam_ctx->buf_size 
    */    
    if(mxc_dma_buffer_init(cam_ctx) != 0)
    {
        err_print("alloc memory for buffer_virtual  %d bytes fail!", g_camera_context->buf_size);
        camera_deinit(g_camera_context);
        mxc_camera_mem_deinit();
	prp_enc_deselect(&ipu_cam_api);
	prp_still_deselect(&ipu_cam_api);
#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
        prp_vf_sdc_deselect(&ipu_cam_api);
#endif
#ifdef CONFIG_MXC_IPU_PRP_VF_ADC
        prp_vf_adc_deselect(&ipu_cam_api);
#endif
	camera_turn_off_vcam();
	video_exclusive_release(inode, file);
        return -ENOMEM;
    }
     
    /*
      set default size and capture format
      init function of each sensor should set proper value 
      for capture_width, capture_height, etc. of camera context 
    */    
    if(camera_set_capture_format(cam_ctx, 1) != 0)
    {
        err_print("camera function init error! capture format!");
        camera_deinit(g_camera_context);
        mxc_camera_mem_deinit();
	prp_enc_deselect(&ipu_cam_api);
	prp_still_deselect(&ipu_cam_api);
#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
        prp_vf_sdc_deselect(&ipu_cam_api);
#endif
#ifdef CONFIG_MXC_IPU_PRP_VF_ADC
        prp_vf_adc_deselect(&ipu_cam_api);
#endif
	camera_turn_off_vcam();
	video_exclusive_release(inode, file);
        return -EIO;
    }

#ifdef CONFIG_MACH_ASCENSION
    /* disable flash by default */
    flash = 0;
    /* initialize timestamp */
    flash_last_on_time = jiffies;
#endif

    dbg_print("MXC_CAMERA: mxc_camera_open success!");
    return 0;
}

static int mxc_camera_close(struct inode *inode, struct file *file)
{
  /* NOTE: this handles the case when the driver is forcibly closed */
  if(g_camera_context->dma_started == 1)
    {
      stop_dma_transfer(g_camera_context);
    }

    camera_deinit(g_camera_context);
    mxc_camera_mem_deinit();
    prp_enc_deselect(&ipu_cam_api);
    prp_still_deselect(&ipu_cam_api);
#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
    prp_vf_sdc_deselect(&ipu_cam_api);
#endif
#ifdef CONFIG_MXC_IPU_PRP_VF_ADC
    prp_vf_adc_deselect(&ipu_cam_api);
#endif

    // Disable power to camera
    camera_turn_off_vcam();

    video_exclusive_release(inode, file);

    dbg_print("MXC_CAMERA: mxc_camera_close\n");
    return 0;
}

static ssize_t mxc_camera_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	/* NOTE: mxc_camera_read() is no longer functional.  
	         The MXC_CAMERA_BUFFER_COPY_TO_USER
	         macro requires virtual addresses, which are
	         no longer available. */
	printk("mxc_camera_read() is not functional\n");

	return 0;
}

static int mxc_camera_WCAM_VIDIOCGBUFINFO(p_camera_context_t cam_ctx, void * param)
{
    buf_info_t buf_info;
    ddbg_print("WCAM_VIDIOCGBUFINFO");

    buf_info.buf_size = cam_ctx->buf_size;
    buf_info.num_frames = cam_ctx->block_number;

    if(copy_to_user(param, &buf_info, sizeof(buf_info_t)))
    {
        return  -EFAULT;
    }
    return 0;
}

static int mxc_camera_WCAM_VIDIOCSPIXFORMAT(p_camera_context_t cam_ctx, void * param)
{
    pix_format_t pix_format;
    ddbg_print("WCAM_VIDIOCSPIXFORMAT");

    if(copy_from_user(&pix_format, param, sizeof(pix_format_t))) 
    {
        return  -EFAULT;
    }
    ddbg_print("capture in %d, out %d", pix_format.vid_in_format, pix_format.vid_out_format);
    ddbg_print("still in %d, out %d", pix_format.still_in_format, pix_format.still_out_format);

    cam_ctx->capture_input_format = pix_format.vid_in_format;
    cam_ctx->capture_output_format = pix_format.vid_out_format;
    cam_ctx->still_input_format = pix_format.still_in_format;
    cam_ctx->still_output_format = pix_format.still_out_format;

    return 0;
}

static int mxc_camera_WCAM_VIDIOCSTARTSTOP(p_camera_context_t cam_ctx, void * param)
{
    int capture_flag = (int)param;
    int result = -EFAULT;
    ddbg_print("WCAM_VIDIOCSTARTSTOP");
    if(capture_flag == STILL_IMAGE) 
    {			
        dbg_print("Still Image capture!");
        stop_dma_transfer(cam_ctx);
        result = camera_capture_still_image(cam_ctx, 0);
    }
    else if(capture_flag == VIDEO_START) 
    {
        dbg_print("Video Image capture!");
        result = camera_start_video_capture(cam_ctx, 0, 1);
    }
    else if(capture_flag == RETURN_VIDEO)
    {
        dbg_print("Return to Video capture!");
        stop_dma_transfer(cam_ctx);
        result = camera_start_video_capture(cam_ctx, 0, 0);
    }
    else if(capture_flag == VIDEO_STOP) 
    {
        dbg_print("Capture stop!"); 
        camera_stop_video_capture(cam_ctx);
        result = 0;
    }
    return result;
}

static int mxc_cam_WCAM_VIDIOCNEXTFRAME(p_camera_context_t cam_ctx, void *param)
{
    int skipFrame;
    if(copy_from_user(&skipFrame, param, sizeof(int)))
    {
        return  -EFAULT;
    }
    cam_ctx->block_tail = (cam_ctx->block_tail + skipFrame) % cam_ctx->block_number;
    return 0;
}

/*get sensor size */
static int mxc_cam_WCAM_VIDIOCGSSIZE(p_camera_context_t cam_ctx, void * param)
{
   window_size_t size;
   ddbg_print("WCAM_VIDIOCGSSIZE");  
   size.w = cam_ctx->sensor_width;
   size.h = cam_ctx->sensor_height;

   if(copy_to_user(param, &size, sizeof(window_size_t)))
   {
       return -EFAULT;
   }
  return 0;
}
         
/*get output size*/
static int mxc_cam_WCAM_VIDIOCGOSIZE(p_camera_context_t cam_ctx, void * param)
{

   window_size_t size;
   ddbg_print("WCAM_VIDIOCGOSIZE");  
   size.w = cam_ctx->capture_width;
   size.h = cam_ctx->capture_height;
   if(copy_to_user(param, &size, sizeof(window_size_t)))
   {
       return -EFAULT;
   }
   return 0;
}
/*get still size*/
static int mxc_cam_WCAM_VIDIOCGCSIZE(p_camera_context_t cam_ctx, void * param)
{

   window_size_t size;
   ddbg_print("WCAM_VIDIOCGCSIZE");
   size.w = cam_ctx->still_width;
   size.h = cam_ctx->still_height;
   if(copy_to_user(param, &size, sizeof(window_size_t)))
   {
       return -EFAULT;
   }
   return 0;
}
/*set still size*/
static int mxc_cam_WCAM_VIDIOCSCSIZE(p_camera_context_t cam_ctx, void * param)
{

   window_size_t size;
   ddbg_print("WCAM_VIDIOCSCSIZE");
   if(copy_from_user(&size, param, sizeof(window_size_t)))
   {
       return -EFAULT;
   }
    if(size.w > cam_ctx->capability.max_width  ||
       size.h > cam_ctx->capability.max_height ||
       size.w > MAX_PIXELS_PER_LINE  ||
       size.h > MAX_LINES_PER_FRAME ||
       size.w < cam_ctx->capability.min_width   ||
       size.h < cam_ctx->capability.min_height)
    {
        err_print("WCAM_VIDIOCSCSIZE error parameter!");
        dbg_print("size.w:%d, MAX_WIDTH:%d, MIN_WIDTH:%d", size.w, 
                  cam_ctx->capability.max_width, 
                  cam_ctx->capability.min_width);
        dbg_print("size.h:%d, MAX_HEIGHT:%d, MIN_HEIGHT:%d", size.h, 
                  cam_ctx->capability.max_height, 
                  cam_ctx->capability.min_height);
        return  -EFAULT;
    }

    //make it in an even multiple of 8

    cam_ctx->still_width  = (size.w+7)/8;
    cam_ctx->still_width  *= 8;

    cam_ctx->still_height = (size.h+7)/8;
    cam_ctx->still_height *= 8;

   return 0;
}


/*set frame buffer count*/
static int mxc_cam_WCAM_VIDIOCSBUFCOUNT(p_camera_context_t cam_ctx, void * param)
{
   int count;
   ddbg_print("");
   if(copy_from_user(&count, param, sizeof(int)))
   {
     return -EFAULT;
   }
   
   if(cam_ctx->block_number_max == 0)
   {
     err_print("windows size or format not setting!!");
     return -EFAULT;
   }
   if(count > MAX_FRAMES)
   {
      count = MAX_FRAMES;
   }
   if(count < FRAMES_IN_BUFFER)
   {
      count = FRAMES_IN_BUFFER;
   }
   
   if(count > cam_ctx->block_number_max)
   {
      count = cam_ctx->block_number_max;
   }
      

   cam_ctx->preferred_block_num = count;
     
   if(copy_to_user(param, &count, sizeof(int)))
   {
     return -EFAULT;
   }
   
   return 0;
}

/*get frame buffer count*/
static int mxc_cam_WCAM_VIDIOCGBUFCOUNT(p_camera_context_t cam_ctx, void * param)
{
   ddbg_print("");
   if(copy_to_user(param, &(cam_ctx->block_number), sizeof(int)))
   {
     return -EFAULT;
   }
   return 0;
}

/*grab the current frame*/
static int mxc_cam_WCAM_VIDIOCGRABFRAME(p_camera_context_t cam_ctx, void * param)
{
  IMAGE_FRAME_T Frame;

  /* this is a critical region because cam_ctx->task_waiting is shared
     between process and interrupt contexts */
  preempt_disable();
  if(cam_ctx->block_header == cam_ctx->block_tail)
    {
      cam_ctx->task_waiting = 1;
      preempt_enable();
      wait_event_interruptible(camera_wait_q, cam_ctx->task_waiting == 0);
    }
  else
    {
      preempt_enable();
    }

  if(last_error != CAMERA_ERROR_NONE)
  {
      err_print("--------ERROR: Some Error Occurred, number = %d--------------", last_error);
      last_error = CAMERA_ERROR_NONE;
  }
#ifdef LOG_TIME_STAMP   //If defined, the time stamp log will be printed out
  if(first_frame == 1)
  {
     camera_capture_time_log(cam_ctx->still_image_mode);
     first_frame = 0;
  }
#endif

  if(cam_ctx->still_image_mode)
  {
      Frame.width = cam_ctx->still_width;
      Frame.height = cam_ctx->still_height;
      Frame.format = cam_ctx->still_output_format;
      
      /* NOTE: images are captured in interleaved format due to IPU,
               so de-interleaving needs to be done if app asks for
               planar format */
      if(cam_ctx->still_output_format == CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR)
	{
	  if(deinterleave(cam_ctx))
	    return -ENOMEM;
	}
  }
  else
  {
      Frame.width = cam_ctx->capture_width;
      Frame.height = cam_ctx->capture_height;
      Frame.format = cam_ctx->capture_output_format;
  }
  Frame.planeNum = cam_ctx->plane_number;
  Frame.first = cam_ctx->block_tail;
  Frame.last = cam_ctx->block_header;
  Frame.planeBytes[0] = cam_ctx->planeBytes[cam_ctx->block_tail][0];
  Frame.planeBytes[1] = cam_ctx->planeBytes[cam_ctx->block_tail][1];
  Frame.planeBytes[2] = cam_ctx->planeBytes[cam_ctx->block_tail][2];

  Frame.planeOffset[0] = cam_ctx->planeOffset[cam_ctx->block_tail][0];
  Frame.planeOffset[1] = cam_ctx->planeOffset[cam_ctx->block_tail][1];
  Frame.planeOffset[2] = cam_ctx->planeOffset[cam_ctx->block_tail][2];

  if(copy_to_user(param, &Frame, sizeof(Frame)))
  {
     return -EFAULT;
  }
  return 0;
}

/*get sensor type*/
static int mxc_cam_WCAM_VIDIOCGSTYPE(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("");
  if(copy_to_user(param, &(cam_ctx->sensor_type), sizeof(cam_ctx->sensor_type)))
  {
    return -EFAULT;
  }
  return 0;
}

/*get capture digital zoom*/
static int mxc_cam_WCAM_VIDIOCGZOOM(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("WCAM_VIDIOCGZOOM");
  if(copy_to_user(param, &(cam_ctx->capture_digital_zoom), sizeof(cam_ctx->capture_digital_zoom)))
  {
    return -EFAULT;
  }
  return 0;
}

/*set capture idigital zoom*/
static int mxc_cam_WCAM_VIDIOCSZOOM(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("WCAM_VIDIOCSZOOM");
  cam_ctx->capture_digital_zoom = (unsigned int)param;
  return 0;
}

/*get still digital zoom*/
static int mxc_cam_WCAM_VIDIOCGSZOOM(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("WCAM_VIDIOCGSZOOM");
  if(copy_to_user(param, &(cam_ctx->still_digital_zoom), sizeof(cam_ctx->still_digital_zoom)))
  {
    return -EFAULT;
  }
  return 0;
}

/*set capture digital zoom*/
static int mxc_cam_WCAM_VIDIOCSSZOOM(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("WCAM_VIDIOCSSZOOM");
  cam_ctx->still_digital_zoom = (unsigned int)param;
  return 0;
}

/*set JPEG quality*/
static int mxc_cam_WCAM_VIDIOCSJPEGQUALITY(p_camera_context_t cam_ctx, void * param)
{
  ddbg_print("WCAM_VIDIOCSJPEGQUALITY");
  cam_ctx->jpeg_quality = (int)param;
  return cam_ctx->camera_functions->command(cam_ctx, WCAM_VIDIOCSJPEGQUALITY, param);
}

#ifdef CONFIG_MACH_ASCENSION
static int do_WCAM_VIDIOCSSTROBEFLASH(void *param)
{
  dbg_print("WCAM_VIDIOCSSTROBEFLASH");
  flash = (int)param;
  return 0;
}
#endif

#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
static int mxc_vf_WCAM_VIDIOCSVFPARAM(p_camera_context_t cam_ctx, void *param)
{
  unsigned int smaller, larger;
#ifdef CONFIG_FB_MXC_OVERLAY
  struct fb_info *fbi = registered_fb[FB_OVL_DEV_INDEX];
#else
  struct fb_info *fbi = registered_fb[FB_DEV_INDEX];
#endif /* CONFIG_FB_MXC_OVERLAY */

  dbg_print("WCAM_VIDIOCSVFPARAM");

  if(copy_from_user(&vfparam, param, sizeof(V4l_VF_PARAM))) {
    return  -EFAULT;
  }

  vfparam.width = (unsigned int)(vfparam.width+7)/8;
  vfparam.width *= 8;
  vfparam.height =  (unsigned int)(vfparam.height+7)/8;
  vfparam.height *= 8;
  smaller = (fbi->var.xres > fbi->var.yres)?fbi->var.yres:fbi->var.xres;
  larger  = (fbi->var.xres > fbi->var.yres)?fbi->var.xres:fbi->var.yres;
  if(((vfparam.width > smaller) && (vfparam.height > smaller)) ||
     (vfparam.width > larger) || (vfparam.height > larger)) {
    ddbg_print("WCAM_VIDIOCSVFPARAM:Invalid parameters");
    return -EINVAL;
  }
  dbg_print("VIDIOCSVFPARAM screen shorter side = %d, longer side = %d", smaller, larger);
  dbg_print("VIDIOCSVFPARAM vf_offset(%d,%d)", vfparam.xoffset, vfparam.yoffset);
  dbg_print("VIDIOCSVFPARAM vf_size(%d,%d)", vfparam.width, vfparam.height);
  dbg_print("VIDIOCSVFPARAM vf_rotation(%d)", vfparam.rotation);
  cam_ctx->vf_configured = true;

  return 0;
}
#endif /* CONFIG_MXC_IPU_PRP_VF_SDC */

static int mxc_camera_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
  void *param = (void *)arg;

   	switch (cmd) 
    {
    /*grab the current frame*/
    case WCAM_VIDIOCGRABFRAME:
         return mxc_cam_WCAM_VIDIOCGRABFRAME(g_camera_context, param);

    /*move to the next frame*/
    case WCAM_VIDIOCNEXTFRAME:
         return mxc_cam_WCAM_VIDIOCNEXTFRAME(g_camera_context, param);

    /*get sensor size */  
    case WCAM_VIDIOCGSSIZE:
         return mxc_cam_WCAM_VIDIOCGSSIZE(g_camera_context, param);
    
    /*get output size*/
    case WCAM_VIDIOCGOSIZE:
         return mxc_cam_WCAM_VIDIOCGOSIZE(g_camera_context, param);

    /*set frame buffer count*/     
    case WCAM_VIDIOCSBUFCOUNT:
         return mxc_cam_WCAM_VIDIOCSBUFCOUNT(g_camera_context, param);

    /*get frame buffer count*/
    case WCAM_VIDIOCGBUFCOUNT:
         return mxc_cam_WCAM_VIDIOCGBUFCOUNT(g_camera_context, param);
         
    /*get cur sensor type*/
    case WCAM_VIDIOCGSTYPE:
         return mxc_cam_WCAM_VIDIOCGSTYPE(g_camera_context, param);

    /*set still mode size*/
    case WCAM_VIDIOCSCSIZE:
         return mxc_cam_WCAM_VIDIOCSCSIZE(g_camera_context, param);

    /*get still mode size*/
    case WCAM_VIDIOCGCSIZE:
         return mxc_cam_WCAM_VIDIOCGCSIZE(g_camera_context, param);

    /*set capture mode digital zoom number*/
    case WCAM_VIDIOCSZOOM:
         return mxc_cam_WCAM_VIDIOCSZOOM(g_camera_context, param);

    /*get capture mode digital zoom number*/
    case WCAM_VIDIOCGZOOM:
         return mxc_cam_WCAM_VIDIOCGZOOM(g_camera_context, param);

    /*set still mode digital zoom number*/
    case WCAM_VIDIOCSSZOOM:
         return mxc_cam_WCAM_VIDIOCSSZOOM(g_camera_context, param);

    /*get still mode digital zoom number*/
    case WCAM_VIDIOCGSZOOM:
         return mxc_cam_WCAM_VIDIOCGSZOOM(g_camera_context, param);

    /*set jpeg quality*/
    case WCAM_VIDIOCSJPEGQUALITY:
         return mxc_cam_WCAM_VIDIOCSJPEGQUALITY(g_camera_context, param);

#ifdef CONFIG_MACH_ASCENSION
    case WCAM_VIDIOCSSTROBEFLASH:
      return do_WCAM_VIDIOCSSTROBEFLASH(param);
#endif

#ifdef CONFIG_MXC_IPU_PRP_VF_SDC
    case WCAM_VIDIOCSVFPARAM:
      return mxc_vf_WCAM_VIDIOCSVFPARAM(g_camera_context, param);
#endif /* CONFIG_MXC_IPU_PRP_VF_SDC */

    /* Set pixel format */
    case WCAM_VIDIOCSPIXFORMAT:
         return mxc_camera_WCAM_VIDIOCSPIXFORMAT(g_camera_context, param);

    /* mmap interface */
    case WCAM_VIDIOCGBUFINFO:
         return mxc_camera_WCAM_VIDIOCGBUFINFO(g_camera_context, param);

    /*start/stop capture */
    case WCAM_VIDIOCSTARTSTOP:
        return mxc_camera_WCAM_VIDIOCSTARTSTOP(g_camera_context, param);

    default:
         return  g_camera_context->camera_functions->command(g_camera_context, cmd, param);
    }
    
   return 0;
}

static int mxc_camera_mmap(struct file *file, struct vm_area_struct *vma)
{
   	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	camera_context_t *cam_ctx = g_camera_context;
	struct page **p_page = cam_ctx->page_array;

#if DEBUG
	printk("camera_mmap: vm_start=0x%x\n", vma->vm_start);
	printk("camera_mmap: vm_end=0x%x\n", vma->vm_end);
#endif

	size = PAGE_ALIGN(size);
	while (size > 0) 
    {
		if(remap_pfn_range(vma, start, \
((u32)(*p_page))>>PAGE_SHIFT, PAGE_SIZE, pgprot_noncached(vma->vm_page_prot)))
        {
			return -EFAULT;
		}
		start += PAGE_SIZE;
		p_page++;
		size -= PAGE_SIZE;
	}
	return 0;
 }

static unsigned int mxc_camera_poll(struct file *file, poll_table *wait) 
{
    static int waited = 0;
    camera_context_t *cam_ctx = g_camera_context;

    poll_wait(file, &camera_wait_q, wait);
    
    if(cam_ctx->still_image_mode == 1 && cam_ctx->still_image_rdy == 1) 
    {
        cam_ctx->still_image_rdy = 0;
        waited = 0;
        return POLLIN | POLLRDNORM;
    }

    /* critical region because cam_ctx->task_waiting is shared
       between process and interrupt contexts */
    preempt_disable();
    if(cam_ctx->block_header == cam_ctx->block_tail)
    {
        cam_ctx->task_waiting = 1;
        waited = 1;
	preempt_enable();
        return 0;
    }
    else
      {
	preempt_enable();
      }

    waited = 0;
    return POLLIN | POLLRDNORM;
}

static int mxc_camera_mem_deinit(void)
{
    if(g_camera_context)
    {
       mxc_dma_buffer_free(g_camera_context);		     

       kfree(g_camera_context);
       g_camera_context = NULL;
    }
    
    return 0;
}

static int mxc_camera_mem_init(void)
{
   g_camera_context = kmalloc(sizeof(struct camera_context_s), GFP_KERNEL);

    if(g_camera_context == NULL)
    {
    	err_print( "MXC_CAMERA: Cann't allocate buffer for camera control structure \n");
        return -ENOMEM;
    }
	
    memset(g_camera_context, 0, sizeof(struct camera_context_s));
    
    ddbg_print("success!"); 
    return 0;
}

/* exists only for sysfs support */
static void camera_sysfs_release(struct video_device *vfd)
{

}

static int __init mxc_camera_init(void)
{
    dbg_print ("enter \n");
    
    /* NOTE: change minor to 1 to avoid conflict with V4L2 device */
    minor =1 ;
 
    if(video_register_device(&vd, VFL_TYPE_GRABBER, minor) < 0) 
    {
         err_print("MXC_CAMERA: video_register_device failed\n");
         return -EIO;
    }

    // initialize camera lock
    camera_lock_init();

#ifdef CONFIG_MXC_IPU_CAMERA_SENSOR2M
    camera_func_sensor2m_s9();
#endif

    dbg_print("MXC_CAMERA: video_register_device successfully. /dev/video%d \n",minor);
  
    return 0;
}

static void __exit mxc_camera_exit(void)
{
    video_unregister_device(&vd);
}

//-------------------------------------------------------------------------------------------------------
//      Configuration APIs
//-------------------------------------------------------------------------------------------------------

static void ci_set_image_format(p_camera_context_t camera_context, int width, int height, int format)
{
  int size = 0;

  ipu_cam_api.v2f.fmt.pix.width = width;
  ipu_cam_api.v2f.fmt.pix.height = height;
  ipu_cam_api.v2f.fmt.pix.pixelformat = palette_to_pixelformat(format);
  switch(ipu_cam_api.v2f.fmt.pix.pixelformat)
    {
    case V4L2_PIX_FMT_RGB565:
      size = width * height * 2;
      break;
    case V4L2_PIX_FMT_BGR24:
      size = width * height * 3;
      break;
    case V4L2_PIX_FMT_RGB24:
      size = width * height * 3;
      break;
    case V4L2_PIX_FMT_BGR32:
      size = width * height * 4;
      break;
    case V4L2_PIX_FMT_RGB32:
      size = width * height * 4;
      break;
    case V4L2_PIX_FMT_YUV422P:
    case V4L2_PIX_FMT_YUYV:
      size = width * height * 2;
      break;
    case V4L2_PIX_FMT_YUV420:
      size = width * height * 3 / 2;
      break;
    default : 
      size = width * height * 4;
      printk("ci_set_image_format: default assume to be YUV444 interleaved.\n");
      break; 
    }

  ipu_cam_api.v2f.fmt.pix.sizeimage = size;

  /* NOTE: must ensure no data transfer activity before reconfiguring parameters */
  stop_dma_transfer(camera_context);

  /* NOTE: reconfigure CSI interface based on new image dimensions and format */
  if(camera_context->still_image_mode)
    {
      /* NOTE: still-image mode */
      ipu_csi_init_interface(ipu_cam_api.v2f.fmt.pix.width*2-1, ipu_cam_api.v2f.fmt.pix.height-1, IPU_PIX_FMT_GENERIC, param);
      ipu_csi_set_window_size(ipu_cam_api.v2f.fmt.pix.width*2, 
                              ipu_cam_api.v2f.fmt.pix.height);

#ifdef CONFIG_MOT_FEAT_2MP_CAMERA_WRKARND
      /* setup workarounds */
      max_slave0_disable();
      enable_gem_clock();
      gem_setup();
      enable_rtic_clock();
      rtic_setup();
      max_rtic_incr_setup();
#endif
    }
  else
    {
      ipu_csi_init_interface(ipu_cam_api.v2f.fmt.pix.width-1, ipu_cam_api.v2f.fmt.pix.height-1, IPU_PIX_FMT_UYVY, param);
  ipu_csi_set_window_size(ipu_cam_api.v2f.fmt.pix.width, 
                          ipu_cam_api.v2f.fmt.pix.height);
    }

  /* Master clock has just been re-enabled; allow settling time */
  mdelay(1);

  /* NOTE: reconfigure DMA channels based on new image dimensions and format.
           These DMA channels are viewfinder-related.  So, reconfiguration is
           not needed for still-image mode. */
  if(!camera_context->still_image_mode)
    {
      prp_enc_setup(&ipu_cam_api);
    }
}

//-------------------------------------------------------------------------------------------------------
//  Control APIs
//-------------------------------------------------------------------------------------------------------
int ci_init()
{
        param.sens_clksrc= 0x0; /* NOTE: external clock */
        param.clk_mode   = 0x2; /* NOTE: CCIR progressive mode */
        param.pixclk_pol = 0x1; /* NOTE: invert pixel clock */
        param.data_width = 0x1; /* NOTE: 8-bit data */
        param.data_pol   = 0x0; /* NOTE: no inversion */
        param.ext_vsync  = 0x0; /* NOTE: ignore external vsync */
        param.Vsync_pol  = 0x0; /* NOTE: no inversion */
        param.Hsync_pol  = 0x0; /* NOTE: no inversion */
     
	ipu_csi_enable_mclk(CSI_MCLK_I2C, true, true);
        set_mclk_rate(&p_mclk_freq);
        ipu_csi_init_interface(200 -1, 160 -1, IPU_PIX_FMT_UYVY, param);
        ipu_csi_set_window_size(200, 160);
	/* Master clock has just been re-enabled; allow settling time */
	mdelay(1);

	return 0;
}

void ci_deinit()
{
  ipu_csi_enable_mclk(ipu_csi_read_mclk_flag(), false, false);
}

void mxc_camera_callback(u32 mask)
{
    camera_context_t  *cam_ctx = g_camera_context;
    int cnt_block;

    if(cam_ctx->still_image_mode == 1) 
    {
        if(cam_ctx->capture_status != CAPTURE_ST_WAIT_DATADONE)
        {
            last_error = CAMERA_ERROR_UNEXPECTEDINT;
            ddbg_print("ERROR:unexpected dma interrupt");
            return;
        }

#ifdef CONFIG_MACH_ASCENSION
	/* NOTE: disable flash if enabled */
	/* ASSUME: assuming lights_fl_update is interrupt-safe */
	if(flash)
	  {
	    lights_fl_update(LIGHTS_FL_APP_CTL_DEFAULT, 1, LIGHTS_FL_REGION_CAMERA_FLASH, LIGHTS_FL_COLOR_BLACK);
	    flash_last_on_time = jiffies;
	  }
#endif

	/* variable "mask" stores overflow error when in still-image mode;
	 * a positive value indicates bayer overflow condition;
	 * so let waiting process timeout and generate error message.
	 */
	if(mask>0)
	  {
	    return;
	  }

        //dbg_print("Stopping still image\n");
        if(cam_ctx->block_number > 1)
            cam_ctx->block_header = (cam_ctx->block_header + 1) % cam_ctx->block_number;
        else
            cam_ctx->block_header++;/*for the max resolution, this is only to notify the buffer is ready*/

        if(cam_ctx->task_waiting == 1) 
        {
            wake_up_interruptible (&camera_wait_q);
            cam_ctx->task_waiting = 0;
        }
        cam_ctx->still_image_rdy = 1;

        cam_ctx->capture_status = CAPTURE_ST_DATADONE;

#ifdef LOG_TIME_STAMP
        do_gettimeofday(&tv6);
#endif
    } 
    else
    {
	/* NOTE: increment frame index to next frame, 
	         unless next frame is tail frame */
	cnt_block = (cam_ctx->block_header + 1) % cam_ctx->block_number;
	if(cnt_block == cam_ctx->block_tail)
	  {
	    cnt_block = cam_ctx->block_header;
	  }

	/* NOTE: setup buffer */
	/* NOTE: this API has no provisions for page-aligned u and v offsets */
	if (ipu_cam_api.enc_update_eba)
	  ipu_cam_api.enc_update_eba((u32)
                (cam_ctx->page_array[cnt_block * cam_ctx->pages_per_block]), 
                &ipu_cam_api.ping_pong_csi);

        cam_ctx->block_header = cnt_block;
    }

    if(cam_ctx->task_waiting == 1 && !(cam_ctx->block_header == cam_ctx->block_tail)) 
    {
        wake_up_interruptible (&camera_wait_q);
        cam_ctx->task_waiting = 0;
    }

}

int deinterleave(p_camera_context_t cam_ctx)
{
  char *image;
  char *ptr;
  char *temp;
  int i, j;
  char *y_ptr;
  char *u_ptr;
  char *v_ptr;

  /* kmalloc might fail with large buffers; use vmalloc instead */
  temp = vmalloc(cam_ctx->still_width*cam_ctx->still_height/2);
  if(temp==NULL)
    {
      printk("CAMERA: not enough memory to do de-interleave\n");
      return -ENOMEM;
    }

  /* map physical addr of image buffer to kernel virtual space */
  /* TODO: this is being done every time this fxn is called.
           At some point, we should make this a one-time mapping
           in an init function. */
  buf_addr_virtual = ioremap((u32) cam_ctx->page_array[0], cam_ctx->buf_size);
  if(buf_addr_virtual == NULL)
    {
      printk("CAMERA: failed to map image buffer to virtual space\n");
      vfree(temp);
      return -ENOMEM;
    }

  /* first stage: de-interleave within half a line
     Cb,Y(i),Cr,Y(i+1)... becomes Y(i),Y(i+1),...Cr...Cb... */

  /* let image point to the first half line of data */
  image = buf_addr_virtual;

  for(j=0;j<cam_ctx->still_height*2;j++)
    {
      memcpy(temp, image, cam_ctx->still_width);
      y_ptr = image;
      u_ptr = y_ptr+cam_ctx->still_width/2;
      v_ptr = u_ptr+cam_ctx->still_width*1/4;
      ptr = temp;
      
      for(i=0;i<cam_ctx->still_width;i+=4)
	{
	  *v_ptr++ = *ptr++;
	  *y_ptr++ = *ptr++;
	  *u_ptr++ = *ptr++;
	  *y_ptr++ = *ptr++;
	}
  
      /* image now points to the next half line */
      image += cam_ctx->still_width;
    }

  /* second stage: de-interleave within pairs of half-lines
     half line i   : Y...Cr...Cb 
     half line i+1 : Y...Cr...Cb
     becomes
     line i   : Y..........
     line i+1 : Cr...Cb... */
  
  /* image now points to the first half line (0,even) */
  image = buf_addr_virtual;

  for(j=0;j<cam_ctx->still_height*2;j+=2)
    {
      /* backup the next half line (odd) */
      memcpy(temp, image+cam_ctx->still_width, cam_ctx->still_width);
      y_ptr = image+cam_ctx->still_width/2;
      u_ptr = image+cam_ctx->still_width;
      v_ptr = u_ptr+cam_ctx->still_width/2;

      ptr = image+cam_ctx->still_width/2;
      memcpy(u_ptr, ptr, cam_ctx->still_width/4);
      u_ptr += cam_ctx->still_width/4;
      ptr += cam_ctx->still_width/4;
      memcpy(v_ptr, ptr, cam_ctx->still_width/4);
      v_ptr += cam_ctx->still_width/4;

      ptr = temp+cam_ctx->still_width/2;
      memcpy(u_ptr, ptr, cam_ctx->still_width/4);
      ptr += cam_ctx->still_width/4;
      memcpy(v_ptr, ptr, cam_ctx->still_width/4);

      memcpy(y_ptr, temp, cam_ctx->still_width/2);

      /* image now points to the next even half-line */
      image += cam_ctx->still_width*2;
    }

  /* third stage: pack the Y components together 
     line i   : Y...
     line i+1 : Cr...Cb...
     line i+2 : Y...
     line i+3 : Cr...Cb...
     becomes
     line i   : Y...
     line i+1 : Y...
     ...
     Cr...Cb...
     ... */

  /* image now points to second line (odd) */
  image = buf_addr_virtual + cam_ctx->still_width;
  ptr = temp;

  for(i=0;i<cam_ctx->still_height/2;i++)
    {
      /* backup odd line */
      memcpy(ptr, image, cam_ctx->still_width);
      ptr += cam_ctx->still_width;

      /* image now points to the next odd line */
      image += cam_ctx->still_width*2;
    }

  /* we now have enough space to pack all the Y components together */

  /* ptr points to second line (odd and empty) */
  ptr = buf_addr_virtual + cam_ctx->still_width;
  /* image points to third line (even and contains Y) */
  image = buf_addr_virtual + cam_ctx->still_width*2;

  for(i=0;i<cam_ctx->still_height-1;i++)
    {
      memcpy(ptr, image, cam_ctx->still_width);
      image += cam_ctx->still_width*2;
      ptr += cam_ctx->still_width;
    }

  /* pack the latter half of Cr and Cb together */
  image = buf_addr_virtual + cam_ctx->still_width*(cam_ctx->still_height*2-3);
  ptr = buf_addr_virtual + cam_ctx->still_width*(cam_ctx->still_height*2-2);

  for(i=0;i<cam_ctx->still_height/2-1;i++)
    {
      memcpy(ptr, image, cam_ctx->still_width);
      image -= cam_ctx->still_width*2;
      ptr -= cam_ctx->still_width;
    }

  /* TODO: optimization can probably be done here.
     Instead of simply restoring Cr and Cb, we can probably combine
     stages 4 and 5 and do them here. */
  /* restore the contents of Cr and Cb in temp buffer */
  memcpy(buf_addr_virtual + cam_ctx->still_width*cam_ctx->still_height, temp, cam_ctx->still_width*cam_ctx->still_height/2);

  /* At this point, Cr and Cb are still full-line interleaved */

  /* stage 4: go from full-line interleaved to double-full-line interleaved */

  /* within 2nd half of buffer, image now points to 2nd line */
  image = buf_addr_virtual + cam_ctx->still_width*cam_ctx->still_height + cam_ctx->still_width;
  ptr = temp;

  for(i=0;i<cam_ctx->still_height/2;i++)
    {
      u_ptr = image - cam_ctx->still_width/2;
      v_ptr = image;

      memcpy(ptr, image, cam_ctx->still_width);
      memcpy(v_ptr, u_ptr, cam_ctx->still_width/2);
      memcpy(u_ptr, ptr, cam_ctx->still_width/2);

      image += cam_ctx->still_width*2;
    }

  /* at this point, Cr and Cb are interleaved on a double-full-line basis */

  /* stage 5: pack Cr and Cb components together */

  image = buf_addr_virtual + cam_ctx->still_width*cam_ctx->still_height + cam_ctx->still_width;
  ptr = temp;

  for(i=0;i<cam_ctx->still_height/2;i++)
    {
      memcpy(ptr, image, cam_ctx->still_width);
      ptr += cam_ctx->still_width;
      image += cam_ctx->still_width*2;
    }

  /* pack Cr together; page-alignment requirement is not met at this point */
  image = buf_addr_virtual + cam_ctx->still_width*cam_ctx->still_height + cam_ctx->still_width*2;
  ptr = buf_addr_virtual + cam_ctx->still_width*cam_ctx->still_height + cam_ctx->still_width;

  for(i=0;i<cam_ctx->still_height/2-1;i++)
    {
      memcpy(ptr, image, cam_ctx->still_width);
      ptr += cam_ctx->still_width;
      image += cam_ctx->still_width*2;
    }

  /* relocate Cr components so that the component order is YCbCr and
     meet page-alignment requirement */
  image = buf_addr_virtual + cam_ctx->still_width*cam_ctx->still_height;
  ptr = buf_addr_virtual + PAGE_ALIGN(cam_ctx->still_width*cam_ctx->still_height) + PAGE_ALIGN(cam_ctx->still_width*cam_ctx->still_height/2);
  memcpy(ptr, image, cam_ctx->still_width*cam_ctx->still_height/2);

  /* restore contents of Cb (already packed) and meet page-alignment requirement */
  ptr = temp;
  image = buf_addr_virtual + PAGE_ALIGN(cam_ctx->still_width*cam_ctx->still_height);
  memcpy(image, ptr, cam_ctx->still_width*cam_ctx->still_height/2);

  iounmap(buf_addr_virtual);
  vfree(temp);
  
  return 0;
}

/* returns 0 for video mode (viewfinder and video capture) and 1 for image capture mode */
int mxc_camera_getmode(void)
{
  return g_camera_context->still_image_mode;
}

/* this function adjusts the number of frames to skip before capture;
   if *frame_cnt = 1, then 2 (=3-1) frames will be skipped for sensor to
   adjust to flash lighting as well as trigger the 2MP workarounds;
   if *frame_cnt = 2; then 1 (=3-2) frame will be skipped to allow the 2MP
   workarounds to be triggered.  */
void mxc_camera_adjtime(int *frame_cnt)
{
#ifdef CONFIG_MACH_ASCENSION
  if(flash)
    {
      *frame_cnt = 1;
    }
  else
#endif
    {
      *frame_cnt = 2;
    }
}

/* This is to ensure power ic driver gets initialized before camera driver
 * because camera driver uses a power ic API.
 */
late_initcall(mxc_camera_init);
module_exit(mxc_camera_exit);

MODULE_DESCRIPTION("Camera Interface driver");
MODULE_LICENSE("GPL");

