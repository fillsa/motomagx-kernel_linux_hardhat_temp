/*
 * Copyright 2004-2005 Freescale Semiconductor, Inc.
 * Copyright 2006-2007 Motorola Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * Date     Author    Comment
 * 10/2006  Motorola  Initial version.  Added the panel initialization and 
 *                    mode switching functions for E02 HVGA displays.  Also 
 *                    added support for changing color mode, RAM window, 
 *                    and partial mode.
 * 11/2006  Motorola  Added mode 4 and mode 6 support
 * 11/2006  Motorola  Fix for Serializer lockup when entering and exiting DSM
 * 12/2006  Motorola  Fixed DSM wakeup switch to line inversion
 * 12/2006  Motorola  Configure the internal-osc freq in mode-6 to lowervalue
 * 01/2007  Motorola  Tightly couple Pixel-clk and serializer control with DMA control
 * 02/2007  Motorola  Refactored common base driver to support multiple panels
 * 03/2007  Motorola  Updated callback function list
 * 04/2007  Motorola  Added image rotation callback handler
 * 04/2007  Motorola  Updated to latest command sequences and fixed timing issues.
 * 05/2007  Motorola  Implemented video tearing fix and fixed up DSM partial window
 * 06/2007  Motorola  Fixed video tearing issue with entering static mode on main display
 * 06/2007  Motorola  Fixed issue with ESD routine occuring outside of vsync
 * 06/2007  Motorola  Updated prepare_dma_start callback handler
 */

#include <linux/delay.h>
#include <asm/mot-gpio.h>
#include "mxcfb_hvga.h"

/*
 * E02 HVGA RGB parameters
 * */
static const struct panel_info E02_hvga_panel = {
	.name                   = "E02 HVGA Panel",
	.type                   = IPU_PANEL_TFT,
	.refresh_rate		= REFRESH_RATE,
	.pixel_fmt              = IPU_PIX_FMT_BGR666,
	.width                  = 240,
	.height                 = 640,
	.top_offset             = 0,
	.left_offset            = 0,
	.middle_porch_lines     = 0,
	.vSyncWidth             = 2,
	.vStartWidth            = 2,
	.vEndWidth              = 60,
	.hSyncWidth             = 8,
	.hStartWidth            = 8,
	.hEndWidth              = 8,
	.sig_pol.datamask_en    = false,
	.sig_pol.clkidle_en     = false,
	.sig_pol.clksel_en      = false,
	.sig_pol.Vsync_pol      = false,
	.sig_pol.enable_pol     = true,
	.sig_pol.data_pol       = false,
	.sig_pol.clk_pol        = true,
	.sig_pol.Hsync_pol      = false,
};

/*
 * E02 QVGA RGB parameters
 * */
static const struct panel_info E02_qvga_panel = {
	.name                   = "E02 QVGA Panel",
	.type                   = IPU_PANEL_TFT,
	.refresh_rate		= REFRESH_RATE,
	.pixel_fmt              = IPU_PIX_FMT_BGR666,
	.width                  = 240,
	.height                 = 320,
	.top_offset             = 0,
	.left_offset            = 0,
	.middle_porch_lines     = 0,
	.vSyncWidth             = 2,
	.vStartWidth            = 2,
	.vEndWidth              = 28,
	.hSyncWidth             = 8,
	.hStartWidth            = 8,
	.hEndWidth              = 8,
	.sig_pol.datamask_en    = false,
	.sig_pol.clkidle_en     = false,
	.sig_pol.clksel_en      = false,
	.sig_pol.Vsync_pol      = false,
	.sig_pol.enable_pol     = true,
	.sig_pol.data_pol       = false,
	.sig_pol.clk_pol        = true,
	.sig_pol.Hsync_pol      = false,
};

static const struct adc_display_conf E02_disp_conf = {
	.write_cycle_time	= 120,
	.write_up_time		= 0,
	.write_down_time	= 60,
	.read_cycle_time	= 120,
	.read_up_time		= 0,
	.read_down_time         = 60,
	.read_latch_time	= 90,
	.read_pixel_clock	= 5000000,
	.pix_fmt		= IPU_PIX_FMT_RGB565,
	.addressing_mode	= XY,
	.vsync_width		= 0,
	.vsync_mode		= VsyncInternal,
	.sig.data_pol		= 0,
	.sig.clk_pol		= 1,
	.sig.cs_pol		= 0,
	.sig.addr_pol		= 0,
	.sig.read_pol		= 0,
	.sig.write_pol		= 0,
	.sig.Vsync_pol		= 0,
	.sig.burst_pol		= 0,
	.sig.burst_mode	        = IPU_ADC_BURST_SERIAL,
	.sig.ifc_mode		= IPU_ADC_IFC_MODE_4WIRE_SERIAL,
	.sig.ifc_width		= 16,
	.sig.ser_preamble_len	= 6,
	.sig.ser_preamble	= 0x1C,
	.sig.ser_rw_mode	= IPU_ADC_SER_RW_AFTER_RS,
};

/*!
 * Function to create and initiate template command buffer for ADC. 
 */
static void E02_init_channel_template(void)
{
	/* template command buffer for ADC is 32*/
	uint32_t tempCmd[TEMPLATE_BUF_SIZE];
	uint32_t i = 0;

	memset(tempCmd, 0, sizeof(uint32_t)*TEMPLATE_BUF_SIZE);

	/************************************************************************/
	/* WRITE Y COORDINATE CMND
	 *
	 * For E02, do not write Y-coordinates every line since the panel 
	 * expects offsets of 320.  The IPU template mask does not allow
	 * for offsets, so we simply initialize 0x201 at the beginning of 
	 * mode 6 and let the panel auto-increment the Y-address.
	 */
	/* tempCmd[i++] = ipu_adc_template_gen(WR_CMND, 0, SINGLE_STEP, 0x201); */
	/* WRITE Y START ADDRESS CMND LSB[22:8] */
	/* tempCmd[i++] = ipu_adc_template_gen(WR_YADDR, 1, SINGLE_STEP, 0x01); */
	/************************************************************************/
	/* Similarly, don't write X coordinates since they are auto-incremented */
	/* WRITE X COORDINATE CMND */
	/* tempCmd[i++] = ipu_adc_template_gen(WR_CMND, 0, SINGLE_STEP, 0x200); */
	/* WRITE X ADDRESS CMND LSB[7:0] */
	/* tempCmd[i++] = ipu_adc_template_gen(WR_XADDR, 1, SINGLE_STEP, 0x01); */
	/************************************************************************/
	tempCmd[i++] = ipu_adc_template_gen(WR_CMND, 0, SINGLE_STEP, 0x202);    
	/* WRITE DATA CMND and STOP */
	tempCmd[i++] = ipu_adc_template_gen(WR_DATA, 1, STOP, 0);               

	ipu_adc_write_template(mxcfb_get_disp_num(), tempCmd, true);
}

static bool spi_mode = false;

static void E02_rotate_image(image_rotate_t main_rotation, image_rotate_t cli_rotation)
{
	uint32_t reg;

	reg = (main_rotation == IMAGE_ROTATE_0) ? 0x30 : 0x0;
	reg |= (cli_rotation == IMAGE_ROTATE_0) ? 0x0 : 0x6;
	reg_write(0x0003, reg);
}
static void _E02_rotate_image(void)
{
	E02_rotate_image(mxcfb_global_state.main_rotation_value, mxcfb_global_state.cli_rotation_value);
}

static const struct panel_init_commands md_off[] = {
	{ 0x0052, 0x0000, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands osc_on[] = {
	{ 0x0000, 0x0001, CMD, 1 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands daa_setting_initialize[] = {
	{ 0x0001, 0x0003, CMD, 0 },
	{ 0x0002, 0x0200, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands daa_setting_finalize[] = {
	{ 0x0007, 0x4040, CMD, 0 },
	{ 0x0008, 0x01A2, CMD, 0 },
	{ 0x0009, 0x0000, CMD, 0 },
	{ 0x000C, 0x0000, CMD, 0 },
	{ 0x000D, 0x000C, CMD, 0 },
	{ 0x000F, 0x0002, CMD, 0 },
	{ 0x0012, 0x0002, CMD, 0 },
	{ 0x0013, 0x0000, CMD, 0 },
	{ 0x0014, 0x0001, CMD, 0 },
	{ 0x0017, 0x0002, CMD, 0 },
	{ 0x0018, 0x0106, CMD, 0 },
	{ 0x0019, 0x1608, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands dbb_setting[] = {
	{ 0x0100, 0x7FFD, CMD, 0 },
	{ 0x0101, 0x0007, CMD, 0 },
	{ 0x0103, 0x0000, CMD, 0 },
	{ 0x0104, 0x0001, CMD, 0 },
	{ 0x0105, 0x007F, CMD, 0 },
	{ 0x0106, 0x0751, CMD, 0 },
	{ 0x0107, 0x0075, CMD, 0 },
	{ 0x0108, 0x2618, CMD, 0 },
	{ 0x0109, 0x2716, CMD, 0 },
	{ 0x010A, 0x0F00, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands dcc_setting[] = {
	{ 0x0300, 0x0001, CMD, 0 },
	{ 0x0301, 0xFC86, CMD, 0 },
	{ 0x0302, 0x8A00, CMD, 0 },
	{ 0x0303, 0x87FC, CMD, 0 },
	{ 0x0304, 0x0001, CMD, 0 },
	{ 0x0305, 0xFC86, CMD, 0 },
	{ 0x0306, 0x8A00, CMD, 0 },
	{ 0x0307, 0x87FC, CMD, 0 },
	{ 0x0308, 0x0001, CMD, 0 },
	{ 0x0309, 0xFC86, CMD, 0 },
	{ 0x030A, 0x8A00, CMD, 0 },
	{ 0x030B, 0x87FC, CMD, 0 },
	{ 0x030C, 0x0002, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands ram_window[] = {
	{ 0x0402, 0x0000, CMD, 0 },
	{ 0x0403, 0x013F, CMD, 0 },
	{ 0x0404, 0x0140, CMD, 0 },
	{ 0x0405, 0x027F, CMD, 0 },
	{ 0x0406, 0x0000, CMD, 0 },
	{ 0x0407, 0x00EF, CMD, 0 },
	{ 0x0408, 0x0000, CMD, 0 },
	{ 0x0409, 0x027F, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands ram_start[] = {
	{ 0x0200, 0x0000, CMD, 0 },
	{ 0x0201, 0x0000, CMD, 0 },
	{ 0x0223, 0x0001, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands ram_start_mode3[] = {
	{ 0x0200, 0x0000, CMD, 0 },
	{ 0x0201, 0x0000, CMD, 0 },
	{ 0x0223, 0x0001, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};



static const struct panel_init_commands deg_setting[] = {
	{ 0x0700, 0x0001, CMD, 0 },
	{ 0x0708, 0x0008, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands ram_mode[] = {
	{ 0x0053, 0x0002, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands des_enable[] = {
	{ 0x0075, 0x0001, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands dee_setting[] = {
	{ 0x0102, 0x0001, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands write_data[] = {
	{ 0x0202, 0x0000, CMD_ONLY, 1 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

/* Mode 2 is CLI only */
static const struct panel_init_commands panel_switch_mode1_to_mode2_initialize[] = {
	{ 0x0053, 0x0000, CMD, 20},
	{ 0x0406, 0x0000, CMD, 0 },
	{ 0x0407, 0x00EF, CMD, 0 },
	{ 0x0408, 0x0140, CMD, 0 },
	{ 0x0409, 0x027F, CMD, 0 },
	{ 0x0200, 0x0000, CMD, 0 },
	{ 0x0201, 0x0140, CMD, 20},
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands panel_switch_mode1_to_mode2_finalize[] = {
	{ 0x0053, 0x0002, CMD, 20},
	{ 0x0004, 0x0001, CMD, 0 },
	{ 0x0001, 0x0002, CMD, 0 },
	{ 0x0007, 0x4040, CMD, 0 },
	{ 0x0008, 0x0182, CMD, 0 },
	{ 0x0101, 0x0005, CMD, 0 },
	{ 0x000D, 0x010C, CMD, 0 },
	{ 0x0004, 0x0000, CMD, 20},
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void switch_mode1_to_mode2(void)
{
	display_init(panel_switch_mode1_to_mode2_initialize);
	hvga_reconfigure_channel();
	display_init(panel_switch_mode1_to_mode2_finalize);
}

/* Mode 3 is MAIN only */
static const struct panel_init_commands panel_switch_mode1_to_mode3_initialize[] = {
	{ 0x0053, 0x0000, CMD, 20},
	{ 0x0406, 0x0000, CMD, 0 },
	{ 0x0407, 0x00EF, CMD, 0 },
	{ 0x0408, 0x0000, CMD, 0 },
	{ 0x0409, 0x013F, CMD, 0 },
	{ 0x0200, 0x0000, CMD, 0 },
	{ 0x0201, 0x0000, CMD, 20},
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands panel_switch_mode1_to_mode3_finalize[] = {
	{ 0x0053, 0x0002, CMD, 20},
	{ 0x0004, 0x0001, CMD, 0 },
	{ 0x0001, 0x0001, CMD, 0 },
	{ 0x0007, 0x4040, CMD, 0 },
	{ 0x0008, 0x0182, CMD, 0 },
	{ 0x0101, 0x0006, CMD, 0 },
	{ 0x000D, 0x010C, CMD, 0 },
	{ 0x0004, 0x0000, CMD, 20},
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void switch_mode1_to_mode3(void)
{
	display_init(panel_switch_mode1_to_mode3_initialize);
	hvga_reconfigure_channel();
	display_init(panel_switch_mode1_to_mode3_finalize);
}

static void switch_mode1_to_mode5(void)
{
	refresh_mode_t temp = mxcfb_global_state.refresh_mode;
	reg_write(0x0053, 0x0000);
	msleep(20);
	mxcfb_global_state.refresh_mode = REFRESH_OFF;
	hvga_reconfigure_channel();
	mxcfb_global_state.refresh_mode = temp;
	reg_write(0x0053, 0x0001);
	msleep(20);
}
static void switch_mode2_to_mode4(void)
{
	switch_mode1_to_mode5();
}
static void switch_mode3_to_mode4a(void)
{
	switch_mode1_to_mode5();
}

static const struct panel_init_commands _switch_mode5_to_mode4[] = {
	{ 0x0053, 0x0000, CMD, 20},
	{ 0x0408, 0x0140, CMD, 0 },
	{ 0x0409, 0x027F, CMD, 0 },
	{ 0x0200, 0x0000, CMD, 0 },
	{ 0x0201, 0x0140, CMD, 0 },
	{ 0x0004, 0x0001, CMD, 0 },
	{ 0x0001, 0x0002, CMD, 0 },
	{ 0x0008, 0x0182, CMD, 0 },
	{ 0x0101, 0x0005, CMD, 0 },
	{ 0x000D, 0x010C, CMD, 0 },
	{ 0x0004, 0x0000, CMD, 20},
	{ 0x0053, 0x0001, CMD, 20},
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void switch_mode5_to_mode4(void)
{
	display_init(_switch_mode5_to_mode4);
}

/* Mode 4a is mode 3 with SPI refresh */
static const struct panel_init_commands _switch_mode5_to_mode4a[] = {
	{ 0x0053, 0x0000, CMD, 20},
	{ 0x0408, 0x0000, CMD, 0 },
	{ 0x0409, 0x013F, CMD, 0 },
	{ 0x0200, 0x0000, CMD, 0 },
	{ 0x0201, 0x0000, CMD, 0 },
	{ 0x0004, 0x0001, CMD, 0 },
	{ 0x0001, 0x0002, CMD, 0 },
	{ 0x0008, 0x0182, CMD, 0 },
	{ 0x0101, 0x0006, CMD, 0 },
	{ 0x000D, 0x010C, CMD, 0 },
	{ 0x0004, 0x0000, CMD, 20},
	{ 0x0053, 0x0001, CMD, 20},
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void switch_mode5_to_mode4a(void)
{
	display_init(_switch_mode5_to_mode4a);
}



static void switch_mode4_to_mode1(bool intermediate_step)
{
	refresh_mode_t temp = mxcfb_global_state.refresh_mode;
	reg_write(0x0053, 0x0000);
	msleep(20);
	reg_write(0x0408, 0x0000);
	reg_write(0x0409, 0x027F);
	reg_write(0x0200, 0x0000);
	reg_write(0x0201, 0x0000);
	if (intermediate_step) {
		mxcfb_global_state.refresh_mode = REFRESH_OFF;
		hvga_reconfigure_channel();
		mxcfb_global_state.refresh_mode = temp;
	} else {
		hvga_reconfigure_channel();
	}
	msleep(20);
	reg_write(0x0053, 0x0002);
	msleep(20);
	reg_write(0x0004, 0x0001);
	reg_write(0x0001, 0x0003);
	reg_write(0x0008, 0x01A2);
	reg_write(0x0101, 0x0007);
	reg_write(0x000D, 0x000C);
	reg_write(0x0004, 0x0000);
	msleep(20);
}

static void switch_mode4_to_mode5(void)
{
	switch_mode4_to_mode1(true);
	switch_mode1_to_mode5();
}

static void switch_mode4_to_mode2(void)
{
	reg_write(0x0053, 0x0000);
	msleep(20);
	hvga_reconfigure_channel();
	reg_write(0x0053, 0x0002);
	msleep(20);
}
static void switch_mode4a_to_mode3(void)
{
	switch_mode4_to_mode2();
}

static const struct panel_init_commands _switch_mode1_to_cam1[] = {
	{ 0x0053, 0x0000, CMD, 0 },
	{ 0x0080, 0x0007, CMD, 20},
	{ 0x000C, 0x0010, CMD, 0 },
	{ 0x0101, 0x0007, CMD, 40},
	{ 0x0053, 0x0002, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void switch_mode1_to_cam1(void) 
{
	display_init(_switch_mode1_to_cam1);
}

static const struct panel_init_commands _switch_mode2_to_cam2[] = {
	{ 0x0053, 0x0000, CMD, 0 },
	{ 0x0080, 0x0007, CMD, 20},
	{ 0x000C, 0x0010, CMD, 40},
	{ 0x0053, 0x0002, CMD, 0},
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void switch_mode2_to_cam2(void)
{
	display_init(_switch_mode2_to_cam2);
}

static const struct panel_init_commands _switch_cam2_to_mode2[] = {
	{ 0x0053, 0x0000, CMD, 0 },
	{ 0x0080, 0x0000, CMD, 20},
	{ 0x000C, 0x0000, CMD, 40},
	{ 0x0053, 0x0002, CMD, 0},
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void switch_cam2_to_mode2(void)
{
	display_init(_switch_cam2_to_mode2);
}

static const struct panel_init_commands panel_switch_mode23_to_mode1_initialize[] = {
	{ 0x0053, 0x0000, CMD, 20 },
	{ 0x0408, 0x0000, CMD, 0 },
	{ 0x0409, 0x027F, CMD, 0 },
	{ 0x0200, 0x0000, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands panel_switch_to_mode1_finalize[] = {
	{ 0x0053, 0x0002, CMD, 20},
	{ 0x0004, 0x0001, CMD, 0 },
	{ 0x0001, 0x0003, CMD, 0 },
	{ 0x0008, 0x01A2, CMD, 0 },
	{ 0x0101, 0x0007, CMD, 0 },
	{ 0x000D, 0x000C, CMD, 0 },
	{ 0x0004, 0x0000, CMD, 20},
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void set_ram_start_pointer(void)
{
	if (mxcfb_global_state.refresh_mode == CLI_FULL_REFRESH) {
		reg_write(0x0201, 0x0140);
	} else {
		reg_write(0x0201, 0x0000);
	}
}

static void switch_mode2_to_mode1(void)
{
	display_init(panel_switch_mode23_to_mode1_initialize);
	hvga_disable_channel();
	set_ram_start_pointer();
	hvga_reconfigure_channel();
	display_init(panel_switch_to_mode1_finalize);
}

static void switch_mode3_to_mode1(void)
{
	switch_mode2_to_mode1();
}

static void switch_mode3_to_cam3(void)
{
	display_init(_switch_mode2_to_cam2);
}

static void switch_cam3_to_mode3(void)
{
	display_init(_switch_cam2_to_mode2);
}


static void switch_cam1_to_mode1(void) {
	reg_write(0x0053, 0x0000);
	reg_write(0x0080, 0x0000);
	set_ram_start_pointer();
	hvga_reconfigure_channel();
	msleep(20);
	reg_write(0x000C, 0x0000);
	reg_write(0x0101, 0x0007);
	msleep(40);
	reg_write(0x0053, 0x0002);
};

static void switch_cam1_to_mode5(void)
{
	reg_write(0x0053, 0x0000);
	reg_write(0x0080, 0x0000);
	msleep(20);
	reg_write(0x000C, 0x0010);
	reg_write(0x0101, 0x0007);
	msleep(40);
	reg_write(0x0053, 0x0001);
	hvga_reconfigure_channel();
}

static void switch_mode5_to_cam1(void)
{
	reg_write(0x0053, 0x0000);
	reg_write(0x0080, 0x0007);
	msleep(20);
	hvga_reconfigure_channel();
	reg_write(0x000C, 0x0010);
	reg_write(0x0101, 0x0007);
	msleep(40);
	reg_write(0x0053, 0x0002);
}

static void switch_to_mode1(panel_t new_state, refresh_mode_t new_refresh)
{
	panel_t old_state = mxcfb_global_state.panel_state;
	mxcfb_global_state.panel_state = new_state;
	mxcfb_global_state.refresh_mode = new_refresh;

	if (spi_mode) {
		if (new_refresh != REFRESH_OFF) {
			switch_mode4_to_mode1(false);
			switch_mode1_to_cam1();
			spi_mode = false;
		} else {
			switch_mode4_to_mode5();
		}
	} else {
		if (old_state == CLI_PANEL) {
			switch_cam2_to_mode2();
			switch_mode2_to_mode1();
		} else {
			switch_cam3_to_mode3();
			switch_mode3_to_mode1();
		}
		switch_mode1_to_cam1();
	}
}

static void switch_to_mode2(panel_t new_state, refresh_mode_t new_refresh)
{
	refresh_mode_t old_refresh = mxcfb_global_state.refresh_mode;
	panel_t old_state = mxcfb_global_state.panel_state;
	mxcfb_global_state.panel_state = new_state;
	mxcfb_global_state.refresh_mode = new_refresh;

	BUG_ON(old_state != (MAIN_PANEL | CLI_PANEL));
	BUG_ON(spi_mode && (new_refresh & CLI_FULL_REFRESH));

	if (spi_mode) {
		switch_mode5_to_mode4();
	} else {
		if (new_refresh & CLI_FULL_REFRESH) {
			switch_cam1_to_mode1();
			switch_mode1_to_mode2();
			switch_mode2_to_cam2();
		} else {
			/* Entering SPI mode */
			mxcfb_global_state.refresh_mode = old_refresh;
			switch_cam1_to_mode1();
			mxcfb_global_state.refresh_mode = new_refresh;
			switch_mode1_to_mode5();
			switch_mode5_to_mode4();
			spi_mode = true;
		}
	}
}

static void switch_to_mode3(panel_t new_state, refresh_mode_t new_refresh)
{
	refresh_mode_t old_refresh = mxcfb_global_state.refresh_mode;
	panel_t old_state = mxcfb_global_state.panel_state;
	mxcfb_global_state.panel_state = new_state;
	mxcfb_global_state.refresh_mode = new_refresh;

	BUG_ON(old_state != (MAIN_PANEL | CLI_PANEL));
	BUG_ON(spi_mode && (new_refresh & MAIN_FULL_REFRESH));

	if (spi_mode) {
		switch_mode5_to_mode4a();
	} else {
		if (new_refresh & MAIN_FULL_REFRESH) {
			switch_cam1_to_mode1();
			switch_mode1_to_mode3();
			switch_mode3_to_cam3();
		} else {
			/* Entering SPI mode */
			mxcfb_global_state.refresh_mode = old_refresh;
			switch_cam1_to_mode1();
			mxcfb_global_state.refresh_mode = new_refresh;
			switch_mode1_to_mode5();
			switch_mode5_to_mode4a();
			spi_mode = true;

		}
	}
}

static void turn_panel_off(void)
{
	reg_write(0x0053, 0x0000);
	hvga_reconfigure_channel();
	reg_write(0x0102, 0x0000);
	msleep(100);
	reg_write(0x0000, 0x0000);
	gpio_lcd_serializer_stby(ASSERT_GPIO_SIGNAL);
	gpio_lcd_serializer_reset(ASSERT_GPIO_SIGNAL);
}

static void turn_on_mode1(void)
{
	gpio_lcd_serializer_reset(DEASSERT_GPIO_SIGNAL);
	msleep(20);
	display_init(md_off);
	display_init(des_enable);
	/* 
	 * The mode transition spec suggests reconfiguring the RGBIF later in 
	 * the initialization process, but this has been found experimentally 
	 * to cause flicker.  We've moved RGB reconfiguration here to avoid flicker.
	 */
	hvga_reconfigure_channel();
	display_init(osc_on);
	display_init(daa_setting_initialize);
	_E02_rotate_image();
	display_init(daa_setting_finalize);
	display_init(dbb_setting);
	display_init(dcc_setting);
	display_init(ram_window);
	display_init(ram_start);
	display_init(deg_setting);
	display_init(ram_mode);
	display_init(dee_setting);
	msleep(220);
}

static void E02_init_panel(void)
{
	msleep(1);
	turn_on_mode1();
	switch_mode1_to_cam1();
}

static const struct panel_init_commands daa_setting_mode2_initialize[] = {
	{ 0x0001, 0x0002, CMD, 0 },
	{ 0x0002, 0x0200, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands daa_setting_mode2_finalize[] = {
	{ 0x0007, 0x4040, CMD, 0 },
	{ 0x0008, 0x0182, CMD, 0 },
	{ 0x0009, 0x0000, CMD, 0 },
	{ 0x000C, 0x0000, CMD, 0 },
	{ 0x000D, 0x010C, CMD, 0 },
	{ 0x000F, 0x0002, CMD, 0 },
	{ 0x0012, 0x0002, CMD, 0 },
	{ 0x0013, 0x0000, CMD, 0 },
	{ 0x0014, 0x0001, CMD, 0 },
	{ 0x0017, 0x0002, CMD, 0 },
	{ 0x0018, 0x0106, CMD, 0 },
	{ 0x0019, 0x1608, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands dbb_setting_mode2[] = {
	{ 0x0100, 0x7FFD, CMD, 0 },
	{ 0x0101, 0x0005, CMD, 0 },
	{ 0x0103, 0x0000, CMD, 0 },
	{ 0x0104, 0x0001, CMD, 0 },
	{ 0x0105, 0x007F, CMD, 0 },
	{ 0x0106, 0x0751, CMD, 0 },
	{ 0x0107, 0x0075, CMD, 0 },
	{ 0x0108, 0x2618, CMD, 0 },
	{ 0x0109, 0x2716, CMD, 0 },
	{ 0x010A, 0x0F00, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands ram_window_mode2[] = {
	{ 0x0402, 0x0000, CMD, 0 },
	{ 0x0403, 0x013F, CMD, 0 },
	{ 0x0404, 0x0140, CMD, 0 },
	{ 0x0405, 0x027F, CMD, 0 },
	{ 0x0406, 0x0000, CMD, 0 },
	{ 0x0407, 0x00EF, CMD, 0 },
	{ 0x0408, 0x0140, CMD, 0 },
	{ 0x0409, 0x027F, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands ram_start_mode2[] = {
	{ 0x0200, 0x0000, CMD, 0 },
	{ 0x0201, 0x0140, CMD, 0 },
	{ 0x0223, 0x0001, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static void turn_on_mode2(void)
{
	gpio_lcd_serializer_reset(DEASSERT_GPIO_SIGNAL);
	msleep(20);
	display_init(md_off);
	display_init(des_enable);
	/* 
	 * The mode transition spec suggests reconfiguring the RGBIF later in 
	 * the initialization process, but this has been found experimentally 
	 * to cause flicker.  We've moved RGB reconfiguration here to avoid flicker.
	 */
	hvga_reconfigure_channel();
	display_init(osc_on);
	display_init(daa_setting_mode2_initialize);
	_E02_rotate_image();
	display_init(daa_setting_mode2_finalize);
	display_init(dbb_setting_mode2);
	display_init(dcc_setting);
	display_init(ram_window_mode2);
	display_init(ram_start_mode2);
	display_init(deg_setting);
	display_init(ram_mode);
	display_init(dee_setting);
	msleep(220);
}

static const struct panel_init_commands daa_setting_mode3_initialize[] = {
	{ 0x0001, 0x0001, CMD, 0 },
	{ 0x0002, 0x0200, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands daa_setting_mode3_finalize[] = {
	{ 0x0007, 0x4040, CMD, 0 },
	{ 0x0008, 0x0182, CMD, 0 },
	{ 0x0009, 0x0000, CMD, 0 },
	{ 0x000C, 0x0000, CMD, 0 },
	{ 0x000D, 0x010C, CMD, 0 },
	{ 0x000F, 0x0002, CMD, 0 },
	{ 0x0012, 0x0002, CMD, 0 },
	{ 0x0013, 0x0000, CMD, 0 },
	{ 0x0014, 0x0001, CMD, 0 },
	{ 0x0017, 0x0002, CMD, 0 },
	{ 0x0018, 0x0106, CMD, 0 },
	{ 0x0019, 0x1608, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands dbb_setting_mode3[] = {
	{ 0x0100, 0x7FFD, CMD, 0 },
	{ 0x0101, 0x0006, CMD, 0 },
	{ 0x0103, 0x0000, CMD, 0 },
	{ 0x0104, 0x0001, CMD, 0 },
	{ 0x0105, 0x007F, CMD, 0 },
	{ 0x0106, 0x0751, CMD, 0 },
	{ 0x0107, 0x0075, CMD, 0 },
	{ 0x0108, 0x2618, CMD, 0 },
	{ 0x0109, 0x2716, CMD, 0 },
	{ 0x010A, 0x0F00, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands ram_window_mode3[] = {
	{ 0x0402, 0x0000, CMD, 0 },
	{ 0x0403, 0x013F, CMD, 0 },
	{ 0x0404, 0x0140, CMD, 0 },
	{ 0x0405, 0x027F, CMD, 0 },
	{ 0x0406, 0x0000, CMD, 0 },
	{ 0x0407, 0x00EF, CMD, 0 },
	{ 0x0408, 0x0000, CMD, 0 },
	{ 0x0409, 0x013F, CMD, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static void turn_on_mode3(void)
{
	gpio_lcd_serializer_reset(DEASSERT_GPIO_SIGNAL);
	msleep(20);
	display_init(md_off);
	display_init(des_enable);
	/* 
	 * The mode transition spec suggests reconfiguring the RGBIF later in 
	 * the initialization process, but this has been found experimentally 
	 * to cause flicker.  We've moved RGB reconfiguration here to avoid flicker.
	 */
	hvga_reconfigure_channel();
	display_init(osc_on);
	display_init(daa_setting_mode3_initialize);
	_E02_rotate_image();
	display_init(daa_setting_mode3_finalize);
	display_init(dbb_setting_mode3);
	display_init(dcc_setting);
	display_init(ram_window_mode3);
	display_init(ram_start_mode3);
	display_init(deg_setting);
	display_init(ram_mode);
	display_init(dee_setting);
	msleep(220);
}

static void E02_enter_low_power_mode(struct partial_mode_info *pm_info)
{
	struct panel_init_commands panel_switch_mode4_to_mode6[] = {
		{ 0x0402, 0x0000, CMD, 0 },
		{ 0x0403, 0x013F, CMD, 0 },
		{ 0x0404, 320 + pm_info->start_y, CMD, 0 },
		{ 0x0405, 320 + pm_info->end_y, CMD, 0 },
		{ 0x0406, 0x0000, CMD, 0 },
		{ 0x0407, 0x00EF, CMD, 0 },
		{ 0x0408, 320 + pm_info->start_y, CMD, 0 },
		{ 0x0409, 320 + pm_info->end_y, CMD, 0 },
		{ 0x0200, 0x0000, CMD, 0 },
		{ 0x0201, 320 + pm_info->start_y, CMD, 0 },
		{ 0x0004, 0x0001, CMD, 0 },
		{ 0x0001, 0x0002, CMD, 0 },
		{ 0x0007, 0xC000, CMD, 0 },
		{ 0x0008, 0x0182, CMD, 0 },
		{ 0x0101, 0x0005, CMD, 0 },
		{ 0x000D, 0x040C, CMD, 0 },
		{ 0x0106, 0x0700, CMD, 0 },
		{ 0x0107, 0x0052, CMD, 0 },
		{ 0x0053, 0x0001, CMD, 0 },
		{ 0x0004, 0x0000, CMD, 30},
		{ 0x0000, 0x0000, INVALID, 0 },
	};

	if (!spi_mode) {
		switch_cam2_to_mode2();
		spi_mode = true;
	}
	reg_write(0x0053, 0x0000);
	msleep(30);
	hvga_reconfigure_channel();
	display_init(panel_switch_mode4_to_mode6);

	mxcfb_update_spi_frame();
}

static void E02_update_low_power_mode(struct partial_mode_info *pm_info)
{
	reg_write(0x0402, 0x0000);
	reg_write(0x0403, 0x013F);
	reg_write(0x0404, 320 + pm_info->start_y);
	reg_write(0x0405, 320 + pm_info->end_y);
	reg_write(0x0406, 0x0000);
	reg_write(0x0407, 0x00EF);
	reg_write(0x0408, 320 + pm_info->start_y);
	reg_write(0x0409, 320 + pm_info->end_y);
	reg_write(0x0200, 0x0000);
	reg_write(0x0201, 320 + pm_info->start_y);
	mxcfb_update_spi_frame();
}

static void E02_exit_low_power_mode(void)
{
	reg_write(0x0053, 0x0000);
	msleep(30);
	reg_write(0x0402, 0x0000);
	reg_write(0x0403, 0x013F);
	reg_write(0x0404, 0x0140);
	reg_write(0x0405, 0x027F);
	reg_write(0x0406, 0x0000);
	reg_write(0x0407, 0x00EF);
	reg_write(0x0408, 0x0140);
	reg_write(0x0409, 0x027F);
	reg_write(0x0200, 0x0000);
	reg_write(0x0201, 0x0140);
	msleep(30);
	hvga_reconfigure_channel();
	reg_write(0x0004, 0x0001);
	reg_write(0x0001, 0x0002);
	reg_write(0x0007, 0x4040);
	reg_write(0x0008, 0x0182);
	reg_write(0x0101, 0x0005);
	reg_write(0x000D, 0x010C);
	reg_write(0x0002, 0x0200);
	reg_write(0x0106, 0x0751);
	reg_write(0x0107, 0x0075);
	reg_write(0x0053, 0x0002);
	reg_write(0x0004, 0x0000);
	msleep(240);

	spi_mode = false;
	switch_mode2_to_cam2();
}

static void E02_switch_panel_state(panel_t new_state, refresh_mode_t new_refresh)
{
	panel_t old_state = mxcfb_global_state.panel_state; 

	if (old_state == PANEL_OFF || new_state == PANEL_OFF) {
		mxcfb_global_state.panel_state = new_state;
		mxcfb_global_state.refresh_mode = new_refresh;
	}


	switch (new_state)
	{
		case PANEL_OFF:
			if (old_state == MAIN_PANEL) {
				switch_cam3_to_mode3();
			} else if (old_state == CLI_PANEL) {
				switch_cam2_to_mode2();
			} else if (old_state == (MAIN_PANEL | CLI_PANEL)) {
				switch_cam1_to_mode1();
			} 
			turn_panel_off();
			break;
		case MAIN_PANEL:
			if (old_state == PANEL_OFF) {
				turn_on_mode3();
				switch_mode3_to_cam3();
			} else {
				switch_to_mode3(new_state, new_refresh);
			}
			break;
		case CLI_PANEL:
			if (old_state == PANEL_OFF) {
				turn_on_mode2();
				switch_mode2_to_cam2();
			} else {
				switch_to_mode2(new_state, new_refresh);
			}
			break;
		case (MAIN_PANEL | CLI_PANEL):
			if (old_state == PANEL_OFF) {
				turn_on_mode1();
				switch_mode1_to_cam1();
			} else {
				switch_to_mode1(new_state, new_refresh);
			}
			break;
		default:
			BUG();
	}
	if (get_legal_refresh_mode() == REFRESH_OFF) {
		spi_mode = true;
	} else {
		spi_mode = false;
	}
	mxcfb_global_state.panel_state = new_state;
	mxcfb_global_state.refresh_mode = new_refresh;
}

/* This command resets 0x007h to the default state */
static void E02_set_color_depth(color_depth_t color_depth)
{
	switch (color_depth) {
		case low_power_color:
		case low_power_monochrome:
			{
				reg_write(0x0007, 0xC000);
				break;
			}
		case full_color:
			{
				reg_write(0x0007, 0x4040);
				break;
			}
		default:
			BUG();
			break;
	}
}

static void E02_swap_buffers(void)
{
	hvga_disable_channel();
	if(mxcfb_global_state.buffers_swapped == 0) {
		reg_write(0x0201, 0x0000);
	} else {
		reg_write(0x0201, 0x0140);
	}
	hvga_reconfigure_channel();
}

static void switch_to_spi_mode(refresh_mode_t old_refresh)
{
	refresh_mode_t refresh = mxcfb_global_state.refresh_mode;
	if (spi_mode) {
		return;
	}
	switch (mxcfb_global_state.panel_state) {
		case (MAIN_PANEL | CLI_PANEL):

			mxcfb_global_state.refresh_mode = old_refresh;
			switch_cam1_to_mode1();
			mxcfb_global_state.refresh_mode = refresh;
			switch_mode1_to_mode5();
			break;
		case MAIN_PANEL:
			if (old_refresh == MAIN_FULL_REFRESH) {
				switch_cam3_to_mode3();
				switch_mode3_to_mode4a();
			}
			break;
		case CLI_PANEL:
			if (old_refresh == CLI_FULL_REFRESH) {
				switch_cam2_to_mode2();
				switch_mode2_to_mode4();
			}
			break;
		default:
			BUG();
	}

	spi_mode = true;
}

static void switch_to_main_full_refresh(refresh_mode_t old_refresh)
{
	spi_mode = false;
	switch (mxcfb_global_state.panel_state) {
		case (MAIN_PANEL | CLI_PANEL):
			if (old_refresh == REFRESH_OFF) {
				switch_mode5_to_cam1();
			} else if (old_refresh == (MAIN_FULL_REFRESH | CLI_FULL_REFRESH)) {
				hvga_reconfigure_channel();
			} else if (old_refresh == MAIN_FULL_REFRESH) {
				/* Nothing to do, return */
				return;
                        } else if (old_refresh == CLI_FULL_REFRESH) {
                                BUG();
			} else {
				BUG();
			}
			break;
		case MAIN_PANEL:
			if (old_refresh == REFRESH_OFF) {
				switch_mode4a_to_mode3();
				switch_mode3_to_cam3();
			} else if (old_refresh == MAIN_FULL_REFRESH) {
				/* Nothing to do, return */
				return;
			} else {
				BUG();
			}
			break;
		case CLI_PANEL:
			BUG();
			break;
		default:
			BUG();
	}
}

static void switch_to_cli_full_refresh(refresh_mode_t old_refresh)
{
	spi_mode = false;

	switch (mxcfb_global_state.panel_state) {
		case (MAIN_PANEL | CLI_PANEL):
			if (old_refresh == REFRESH_OFF) {
				switch_mode4_to_mode2();
				switch_mode2_to_mode1();
				switch_mode1_to_cam1();
			} else {
				switch_cam1_to_mode1();
			}
			break;
		case MAIN_PANEL:
			BUG();
			break;
		case CLI_PANEL:
			if (old_refresh == REFRESH_OFF) {
				switch_mode4_to_mode2();
				switch_mode2_to_cam2();
			} else {
				/* Show logo */
				switch_mode4_to_mode2();
				return;
			}
			break;
		default:
			BUG();
	}
}

static void switch_to_hvga_refresh(refresh_mode_t old_refresh)
{
	spi_mode = false;

	switch (mxcfb_global_state.panel_state) {
		case (MAIN_PANEL | CLI_PANEL):
			if (old_refresh == REFRESH_OFF) {
				switch_mode5_to_cam1();
			} else if (old_refresh == MAIN_FULL_REFRESH) {
				hvga_reconfigure_channel();
			} else {
				BUG();
			}
			break;
		default:
			BUG();
	}
}

static void E02_set_refresh_mode(refresh_mode_t refresh)
{
	refresh_mode_t old_refresh = get_legal_refresh_mode();
	mxcfb_global_state.refresh_mode = refresh;

	if (mxcfb_global_state.panel_state == PANEL_OFF) {
		return;
	}

	switch (get_legal_refresh_mode()) {
		case REFRESH_OFF:
			switch_to_spi_mode(old_refresh);
			break;
		case MAIN_FULL_REFRESH:
			switch_to_main_full_refresh(old_refresh);
			break;
		case CLI_FULL_REFRESH:
			switch_to_cli_full_refresh(old_refresh);
			break;
		case (MAIN_FULL_REFRESH | CLI_FULL_REFRESH):
			switch_to_hvga_refresh(old_refresh);
			break;
		default:
			BUG();
	}
	if (refresh == REFRESH_OFF) {
		spi_mode = true;
	} else {
		spi_mode = false;
	}
}

/* Global panel information pointers */
struct panel_specific_functions E02_panel_functions = {
	.disp_conf = &E02_disp_conf,
	.hvga_panel_info = &E02_hvga_panel,
	.qvga_panel_info = &E02_qvga_panel,
	.init_panel = E02_init_panel,
	.init_channel_template = E02_init_channel_template,
	.switch_panel_state = E02_switch_panel_state,
	.enter_low_power_mode = E02_enter_low_power_mode,
	.update_low_power_mode = E02_update_low_power_mode,
	.exit_low_power_mode = E02_exit_low_power_mode,
	.rotate_image = E02_rotate_image,
	.swap_buffers = E02_swap_buffers,
	.set_color_depth = E02_set_color_depth,
	.set_refresh_mode = E02_set_refresh_mode,
	.prepare_dma_start = NULL,
	.prepare_dma_stop_initialize = NULL,
	.prepare_dma_stop_finalize = NULL,
	.custom_command = NULL,
};

