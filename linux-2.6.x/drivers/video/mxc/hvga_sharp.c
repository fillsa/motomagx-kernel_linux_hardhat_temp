/*
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
 *                    mode switching functions for Sharp HVGA displays.  Also 
 *                    added support for changing color mode, RAM window, 
 *                    and partial mode.
 * 02/2007  Motorola  Updated to remove flickers and added DSM
 * 03/2007  Motorola  Updated driver with new Sharp command sequences
 * 04/2007  Motorola  Added image rotation callback handler
 * 04/2007  Motorola  Added ADC SPI template and deserializer reset callback
 * 05/2007  Motorola  Updated DSM sequences
 * 06/2007  Motorola  Fixed video tear on main display
 * 06/2007  Motorola  Updated static mode and prepare_dma_start callback handler
 */

#include <linux/delay.h>
#include <asm/mot-gpio.h>
#include "mxcfb_hvga.h"

/*
 * Sharp HVGA RGB parameters
 * */
static const struct panel_info sharp_hvga_panel = {
	.name                   = "Sharp HVGA Panel",
	.type                   = IPU_PANEL_TFT,
	.refresh_rate           = REFRESH_RATE,
	.pixel_fmt              = IPU_PIX_FMT_BGR666,
	.width                  = 240,
	.height                 = 640,
	.top_offset             = 0,
	.left_offset            = 0,
	.middle_porch_lines     = 2,
	.vSyncWidth             = 2,
	.vStartWidth            = 1,
	.vEndWidth              = 3,
	.hSyncWidth             = 10,
	.hStartWidth            = 60,
	.hEndWidth              = 70,
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
 * Sharp QVGA RGB parameters
 * */
static const struct panel_info sharp_qvga_panel = {
	.name                   = "Sharp QVGA Panel",
	.type                   = IPU_PANEL_TFT,
	.refresh_rate           = REFRESH_RATE,
	.pixel_fmt              = IPU_PIX_FMT_BGR666,
	.width                  = 240,
	.height                 = 320,
	.top_offset             = 0,
	.left_offset            = 0,
	.middle_porch_lines     = 0,
	.vSyncWidth             = 1,
	.vStartWidth            = 2,
	.vEndWidth              = 1,
	.hSyncWidth             = 10,
	.hStartWidth            = 60,
	.hEndWidth              = 70,
	.sig_pol.datamask_en    = false,
	.sig_pol.clkidle_en     = false,
	.sig_pol.clksel_en      = false,
	.sig_pol.Vsync_pol      = false,
	.sig_pol.enable_pol     = true,
	.sig_pol.data_pol       = false,
	.sig_pol.clk_pol        = true,
	.sig_pol.Hsync_pol      = false,
};

static const struct adc_display_conf sharp_disp_conf = {
	.write_cycle_time       = 120,
	.write_up_time          = 0,
	.write_down_time        = 60,
	.read_cycle_time        = 120,
	.read_up_time           = 0,
	.read_down_time         = 60,
	.read_latch_time        = 90,
	.read_pixel_clock       = 5000000,
	.pix_fmt                = IPU_PIX_FMT_RGB565,
	.addressing_mode        = XY,
	.vsync_width            = 0,
	.vsync_mode             = VsyncInternal,
	.sig.data_pol           = 0,
	.sig.clk_pol            = 1,
	.sig.cs_pol             = 0,
	.sig.addr_pol           = 0,
	.sig.read_pol           = 0,
	.sig.write_pol          = 0,
	.sig.Vsync_pol          = 0,
	.sig.burst_pol          = 0,
	.sig.burst_mode         = IPU_ADC_BURST_SERIAL,
	.sig.ifc_mode           = IPU_ADC_IFC_MODE_4WIRE_SERIAL,
	.sig.ifc_width          = 16,
	.sig.ser_preamble_len   = 6,
	.sig.ser_preamble       = 0x1C,
	.sig.ser_rw_mode        = IPU_ADC_SER_RW_AFTER_RS,
};

static void sharp_init_channel_template(void)
{
        /* template command buffer for ADC is 32*/
        uint32_t tempCmd[TEMPLATE_BUF_SIZE];
        uint32_t i = 0;

        memset(tempCmd, 0, sizeof(uint32_t)*TEMPLATE_BUF_SIZE);
        /* setup update display region*/
        /* WRITE Y COORDINATE CMND */
        tempCmd[i++] = ipu_adc_template_gen(WR_CMND, 0, SINGLE_STEP, 0x104);  
        /* WRITE Y START ADDRESS CMND LSB[22:8] */
        tempCmd[i++] = ipu_adc_template_gen(WR_YADDR, 1, SINGLE_STEP, 0x01);    
        /* WRITE X COORDINATE CMND*/
        tempCmd[i++] = ipu_adc_template_gen(WR_CMND, 0, SINGLE_STEP, 0x102);  
        /* WRITE X ADDRESS CMND LSB[7:0] */
        tempCmd[i++] = ipu_adc_template_gen(WR_XADDR, 1, SINGLE_STEP, 0x01);    
        tempCmd[i++] = ipu_adc_template_gen(WR_CMND, 0, SINGLE_STEP, 0x202);    
        /* WRITE DATA CMND and STP */
        tempCmd[i++] = ipu_adc_template_gen(WR_DATA, 1, STOP, 0);               

        ipu_adc_write_template(mxcfb_get_disp_num(), tempCmd, true);
}

static bool spi_mode_enabled = false;

static void sharp_set_refresh_mode(refresh_mode_t);

void tango_reg_write(uint32_t index, uint32_t command)
{
	uint32_t cmd;

	cmd = ((index << 8) | (0xFF & command));

	reg_write(0x0332, cmd);
	reg_write(0x0330, 0x2);

	if (poll_read(0x330, 0x2, 0) < 0) {
		printk("MXCFB Error %s: poll_read timeout on 0x330, index = 0x%X, cmd = 0x%X\n", 
				__func__, index, command & 0xFF);
	}
}

uint32_t tango_reg_read(uint32_t index)
{
	reg_write(0x0332, (index << 8));
	reg_write(0x0330, 0x6);

	if (poll_read(0x0330, 0x2, 0) < 0) {
		printk("MXCFB Error %s: poll_read timeout on 0x330, index = 0x%X\n", 
				__func__, index);
	}

	return reg_read(0x033A);
}

static int sharp_custom_command(int disp, const struct panel_init_commands * command)
{
	if (command->mode == TANGO) {
		tango_reg_write(command->index, command->data);
	} else {
		BUG();
	}

	return 0;
}

static void tango_assert_valtran(void)
{
	tango_reg_write(0x31, 0x01);
	/* 
	 * Valtran does not get asserted until the next vsync.  In order to 
	 * ensure that Valtran has been asserted by the time this function ends,
	 * sleep for VSYNC_WAIT.
	 */
	msleep(VSYNC_WAIT);
}

static void _sharp_rotate_image(bool assert_valtran, image_rotate_t main, 
		image_rotate_t cli)
{
	uint32_t flip;
	tango_reg_write(0xEF, 0x00);
	flip = tango_reg_read(0x34);
	if (mxcfb_global_state.main_rotation_value == IMAGE_ROTATE_0) {
		tango_reg_write(0x34, flip | 0x3);
	} else {
		tango_reg_write(0x34, flip & ~0x3);
	}

	tango_reg_write(0xEF, 0x01);
	flip = tango_reg_read(0x34);
	if (mxcfb_global_state.cli_rotation_value == IMAGE_ROTATE_0) {
		tango_reg_write(0x34, flip | 0x3);
	} else {
		tango_reg_write(0x34, flip & ~0x3);
	}

	if (assert_valtran) {
		tango_assert_valtran();
	}
}

static const struct panel_init_commands tango_init_poweron[] = {
	/* TANGO Initial Setting */
	{ 0xE1, 0x04, TANGO, 0 },
	{ 0xEC, 0x00, TANGO, 0 },
	{ 0xEA, 0x01, TANGO, 0 },
	{ 0xEA, 0x00, TANGO, 0 },
	{ 0xEB, 0x00, TANGO, 0 },
	{ 0xE3, 0x00, TANGO, 0 },
	{ 0xE0, 0x07, TANGO, 6 },
	{ 0xEB, 0x01, TANGO, 0 },
	{ 0xE3, 0x80, TANGO, 0 },
	{ 0xE0, 0x07, TANGO, 6 },
	{ 0xEC, 0x01, TANGO, 0 },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x15, 0x5A, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x15, 0x5A, TANGO, 0 },
	/* TANGO POWER ON */
	{ 0x80, 0x00, TANGO, 0 },
	{ 0x80, 0x01, TANGO, 0 },
	{ 0x80, 0x03, TANGO, 0 },
	{ 0x80, 0x07, TANGO, 0 },
	{ 0x84, 0x03, TANGO, 0 },
	{ 0x80, 0x0F, TANGO, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands tango_turn_on_mode5dash[] =  {
	/* TANGO Display ON */
	{ 0x80, 0x1F, TANGO, 0 },
	{ 0x80, 0x3F, TANGO, 0 },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x35, 0x61, TANGO, 0 },
	{ 0xB0, 0x06, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x35, 0x61, TANGO, 0 },
	{ 0xB0, 0x06, TANGO, 0 },
	{ 0x31, 0x01, TANGO, VSYNC_WAIT },
	{ 0x0340, 0x0001, RODEM, 0 },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0xB0, 0x94, TANGO, 0 },
	{ 0xB1, 0x27, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0xB0, 0x94, TANGO, 0 },
	{ 0xB1, 0x27, TANGO, 0 },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x33, 0x02, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x33, 0x02, TANGO, 0 },
	{ 0x33, 0x03, TANGO, 0 },
	{ 0x31, 0x01, TANGO, VSYNC_WAIT },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x35, 0x00, TANGO, 0 },
	{ 0x30, 0x02, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x35, 0x00, TANGO, 0 },
	{ 0x30, 0x02, TANGO, VSYNC_WAIT },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x30, 0x03, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x30, 0x03, TANGO, VSYNC_WAIT },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands tango_turn_on_mode2dash[] =  {
	/* TANGO Display ON */
	{ 0x80, 0x1F, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x35, 0x61, TANGO, 0 },
	{ 0xB0, 0x06, TANGO, 0 },
	{ 0x31, 0x01, TANGO, VSYNC_WAIT },
	{ 0x0340, 0x0003, RODEM, 0 },
	{ 0xB0, 0x94, TANGO, 0 },
	{ 0xB1, 0x27, TANGO, 0 },
	{ 0x33, 0x02, TANGO, 0 },
	{ 0x33, 0x03, TANGO, 0 },
	{ 0x31, 0x01, TANGO, VSYNC_WAIT },
	{ 0x35, 0x00, TANGO, 0 },
	{ 0x30, 0x02, TANGO, VSYNC_WAIT },
	{ 0x30, 0x03, TANGO, VSYNC_WAIT },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands tango_turn_on_mode3dash[] =  {
	/* TANGO Display ON */
	{ 0x80, 0x2F, TANGO, 0 },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x35, 0x61, TANGO, 0 },
	{ 0xB0, 0x06, TANGO, 0 },
	{ 0x31, 0x01, TANGO, VSYNC_WAIT },
	{ 0x0340, 0x0003, RODEM, 0 },
	{ 0xB0, 0x94, TANGO, 0 },
	{ 0xB1, 0x27, TANGO, 0 },
	{ 0x33, 0x02, TANGO, 0 },
	{ 0x33, 0x03, TANGO, 0 },
	{ 0x31, 0x01, TANGO, VSYNC_WAIT },
	{ 0x35, 0x00, TANGO, 0 },
	{ 0x30, 0x02, TANGO, VSYNC_WAIT },
	{ 0x30, 0x03, TANGO, VSYNC_WAIT },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands rodem_base_settings_initialize[] = {
	/* Base Setting */
	{ 0x0040, 0x0002, RODEM, 1 },
	{ 0x0040, 0x0003, RODEM, 0 },
	{ 0x0002, 0x0010, RODEM, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};
static const struct panel_init_commands rodem_base_settings_finalize[] = {
	{ 0x0020, 0x5019, RODEM, 0 },
	{ 0x0022, 0x1601, RODEM, 0 },
	{ 0x0024, 0x0001, RODEM, 0 },
	{ 0x0026, 0x05FE, RODEM, 0 },
	{ 0x000E, 0x0033, RODEM, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void rodem_set_clkdiv2(int hvga_mode)
{
	if (hvga_mode == HVGA_MODE) {
		reg_write(0x0010, 0x0100);
	} else {
		reg_write(0x0010, 0x1100);
	}
}

static void rodem_set_rgb_timings(int hvga)
{
	reg_write(0x0342, 0x000A);
	reg_write(0x0344, 0x0050);
	reg_write(0x0346, 0x00F0);
	reg_write(0x0348, 0x0032);
	if (hvga == HVGA_MODE) {
		/* External RGB IF Timming Setting */
		reg_write(0x034A, 0x0002);
		reg_write(0x034C, 0x0003);
		reg_write(0x034E, 0x0140);
		reg_write(0x0350, 0x0001);
		reg_write(0x0352, 0x0002);
	} else {
		/* External RGB IF Timming Setting */
		reg_write(0x034A, 0x0001);
		reg_write(0x034C, 0x0001);
		reg_write(0x034E, 0x0140);
		reg_write(0x0350, 0x0002);
	}
}

static const struct panel_init_commands rodem_panel_generator_timings[] = {
	{ 0x042C, 0x000A, RODEM, 0 },
	{ 0x042E, 0x0050, RODEM, 0 },
	{ 0x0430, 0x00F0, RODEM, 0 },
	{ 0x0432, 0x0032, RODEM, 0 },
	{ 0x0434, 0x0002, RODEM, 0 },
	{ 0x0436, 0x0003, RODEM, 0 },
	{ 0x0438, 0x0140, RODEM, 0 },
	{ 0x043A, 0x0001, RODEM, 0 },
	{ 0x043C, 0x0002, RODEM, 0 },
	{ 0x042A, 0x0030, RODEM, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands rodem_set_pixel_area[] = {
	{ 0x0100, 0x0000, RODEM, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void rodem_set_vram_capture_settings(refresh_mode_t refresh_mode)
{
	switch (refresh_mode) {
		case REFRESH_OFF:
			reg_write(0x0280, 0x0000);
			break;
		case MAIN_FULL_REFRESH:
			reg_write(0x0282, 0x0000);
			reg_write(0x0280, 0x0001);
			break;
		case CLI_FULL_REFRESH:
			reg_write(0x0282, 0x0002);
			reg_write(0x0280, 0x0001);
			break;
		case MAIN_FULL_REFRESH | CLI_FULL_REFRESH:
			reg_write(0x0282, 0x0004);
			reg_write(0x0280, 0x0003);
			break;
		default:
			BUG();
	}
}

static void rodem_set_panel_mode(refresh_mode_t refresh_mode)
{
	if (refresh_mode == (MAIN_FULL_REFRESH | CLI_FULL_REFRESH)) {
		reg_write(0x0340, 0x1);
	} else {
		reg_write(0x0340, 0x3);
	}
}

static void rodem_set_clkconf(int hvga) 
{
	reg_write(0x000C, (hvga == HVGA_MODE) ? 0x0308 : 0x0708);
}

static void rodem_set_vdelay(refresh_mode_t refresh_mode, bool eliminate_tearing) 
{
	if (eliminate_tearing) {
		reg_write(0x0364, (refresh_mode == CLI_FULL_REFRESH) ? 0x0080 : 0x0000);
	} else {
		reg_write(0x0364, 0x0300);
	}
}
static void rodem_set_ptvlim(bool eliminate_tearing)
{
	reg_write(0x043E, eliminate_tearing ? 0x01FF : 0x0003);
}


/* Clock Settings */
static const struct panel_init_commands rodem_clock_settings[] = {
	{ 0x0002, 0x0011, RODEM, 0 },
	{ 0x000E, 0x0233, RODEM, 30 },
	{ 0x000E, 0x0333, RODEM, 10 },
	{ 0x0002, 0x0211, RODEM, 0 },
	{ 0x043E, 0x01FF, RODEM, 0 },
	{ 0x0458, 0x5001, RODEM, 0 },
	{ 0x045A, 0x200A, RODEM, 0 },
	{ 0x0364, 0x0000, RODEM, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};



static void rodem_set_display_source(panel_t panel_mode)
{
	switch (panel_mode) {
		case PANEL_OFF:
			break;
		case MAIN_PANEL:
			reg_write(0x0400, 0x0103);
			break;
		case CLI_PANEL:
			reg_write(0x0400, 0x0121);
			break;
		case MAIN_PANEL | CLI_PANEL:
			reg_write(0x0400, 0x0123);
			break;
		default:
			BUG();
	}
}

static void rodem_turn_on_panel(int hvga_mode) 
{
	display_init(rodem_base_settings_initialize);
	rodem_set_clkconf(refresh_mode_to_hvga_mode());
	display_init(rodem_base_settings_finalize);
	rodem_set_clkdiv2(refresh_mode_to_hvga_mode());
	rodem_set_rgb_timings(refresh_mode_to_hvga_mode());
	display_init(rodem_panel_generator_timings);
	display_init(rodem_set_pixel_area);
	rodem_set_vram_capture_settings(get_legal_refresh_mode()); 
	display_init(rodem_clock_settings);
	reg_write(0x0330, 0x0000);
	rodem_set_display_source(mxcfb_global_state.panel_state);
}

static const struct panel_init_commands tango_off_sequence[] = {
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x35, 0x01, TANGO, 0 },
	{ 0x30, 0x02, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x35, 0x01, TANGO, 0 },
	{ 0x30, 0x02, TANGO, VSYNC_WAIT },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x35, 0xC1, TANGO, 0 },
	{ 0x30, 0x00, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x35, 0xC1, TANGO, 0 },
	{ 0x30, 0x00, TANGO, VSYNC_WAIT },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x35, 0x41, TANGO, 0 },
	{ 0x33, 0x02, TANGO, 0 },
	{ 0x33, 0x00, TANGO, 0 },
	{ 0xB0, 0x09, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x35, 0x41, TANGO, 0 },
	{ 0x33, 0x00, TANGO, 0 },
	{ 0xB0, 0x09, TANGO, 0 },
	{ 0x31, 0x01, TANGO, VSYNC_WAIT },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0xB0, 0x00, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0xB0, 0x00, TANGO, 0 },
	{ 0xEF, 0x00, TANGO, 0 },
	{ 0x35, 0x11, TANGO, 0 },
	{ 0xEF, 0x01, TANGO, 0 },
	{ 0x35, 0x11, TANGO, 0 },
	{ 0x31, 0x01, TANGO, VSYNC_WAIT },
	{ 0x80, 0x1F, TANGO, 0 },
	{ 0x80, 0x0F, TANGO, 0 },
	{ 0x80, 0x07, TANGO, 0 },
	{ 0x80, 0x03, TANGO, 0 },
	{ 0x80, 0x01, TANGO, 0 },
	{ 0x80, 0x00, TANGO, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static const struct panel_init_commands rodem_off_sequence[] = {
	{ 0x0400, 0x0000, RODEM, 0 },
	{ 0x0330, 0x0080, RODEM, VSYNC_WAIT },
	{ 0x0002, 0x0011, RODEM, 0 },
	{ 0x000E, 0x0033, RODEM, 0 },
	{ 0x0002, 0x0010, RODEM, 0 },
	{ 0x0340, 0x0000, RODEM, 0 },
	{ 0x0040, 0x0002, RODEM, 0 },
	{ 0x0000, 0x0000, INVALID, 0 },
};

static void enable_serializer(void) 
{
	gpio_lcd_serializer_reset(DEASSERT_GPIO_SIGNAL);
	udelay(20);
	gpio_lcd_serializer_stby(DEASSERT_GPIO_SIGNAL);
	msleep(10);

	hvga_reconfigure_channel();
}

static void turn_on_panel(int hvga_mode)
{
	enable_serializer();
	rodem_turn_on_panel(hvga_mode);
	display_init(tango_init_poweron);
	_sharp_rotate_image(false, mxcfb_global_state.main_rotation_value,
			mxcfb_global_state.cli_rotation_value);
}
	
static void turn_on_mode5dash(void)
{
	turn_on_panel(HVGA_MODE);
	display_init(tango_turn_on_mode5dash);
}

static void turn_on_mode2dash(void)
{
	turn_on_panel(QVGA_MODE);
	display_init(tango_turn_on_mode2dash);
}

static void turn_on_mode3dash(void)
{
	turn_on_panel(QVGA_MODE);
	display_init(tango_turn_on_mode3dash);
}

static void stop_memory_capture(void)
{
	reg_write(0x0280, 0x0000);
}

static void display_rgbif_off(void)
{
	reg_write(0x0340, 0x0000);
}

static void switch_mode23dash_to_mode5dash(panel_t cli_or_main)
{
	stop_memory_capture();
	tango_reg_write(0xEF, (cli_or_main == MAIN_PANEL) ? 0x00 : 0x01);
	reg_write(0x0400, 0x0000);
	msleep(VSYNC_WAIT);
	
	display_rgbif_off();

	/* Enable tearing fix on CLI if flip closed */
	rodem_set_vdelay(get_legal_refresh_mode(), true);

	reg_write(0x0002, 0x0011);
	rodem_set_rgb_timings(refresh_mode_to_hvga_mode());
	reg_write(0x0010, 0x0100);
	reg_write(0x0002, 0x0211);
	
	hvga_reconfigure_channel();

	rodem_set_display_source(mxcfb_global_state.panel_state);
	
	rodem_set_panel_mode(get_legal_refresh_mode());
	/* Sleep for 1ms before starting memory capture to avoid flicker */
	msleep(1);
	rodem_set_vram_capture_settings(get_legal_refresh_mode()); 
	
	tango_reg_write(0x80, 0x3F);
	tango_reg_write(0xEF, (cli_or_main == MAIN_PANEL) ? 0x01 : 0x00);
	tango_reg_write(0x35, 0x61);
	tango_reg_write(0xB0, 0x06);
	tango_reg_write(0x31, 0x01);
	msleep(VSYNC_WAIT);
	tango_reg_write(0xB0, 0x94);
	tango_reg_write(0xB1, 0x27);
	tango_reg_write(0x33, 0x03);
	tango_reg_write(0x31, 0x01);
	msleep(VSYNC_WAIT);
	tango_reg_write(0x35, 0x00);
	tango_reg_write(0x30, 0x02);
	msleep(VSYNC_WAIT);
	tango_reg_write(0x30, 0x03);
	tango_reg_write(0xEF, (cli_or_main == MAIN_PANEL) ? 0x00 : 0x01);
	tango_reg_write(0x30, 0x03);
	msleep(VSYNC_WAIT);
}

static void switch_mode5dash_to_mode23dash(panel_t cli_or_main)
{
	stop_memory_capture();
	tango_reg_write(0xEF, (cli_or_main == MAIN_PANEL) ? 0x01 : 0x00);
	tango_reg_write(0x35, 0x01);
	tango_reg_write(0x30, 0x02);
	msleep(VSYNC_WAIT);
	tango_reg_write(0x35, 0xC1);
	tango_reg_write(0x30, 0x00);
	msleep(VSYNC_WAIT);
	tango_reg_write(0x35, 0x41);
	tango_reg_write(0x33, 0x01);
	tango_reg_write(0xB0, 0x09);
	tango_reg_write(0xB1, 0x27);
	tango_reg_write(0x31, 0x01);
	msleep(VSYNC_WAIT);
	tango_reg_write(0xB0, 0x00);
	tango_reg_write(0x35, 0x11);
	tango_reg_write(0xEF, (cli_or_main == MAIN_PANEL) ? 0x00 : 0x01);
	if (cli_or_main == MAIN_PANEL) {
		tango_reg_write(0x80, tango_reg_read(0x80) & ~(0x10));
	} else {
		tango_reg_write(0x80, tango_reg_read(0x80) & ~(0x20));
	}
	
	reg_write(0x0400, 0x0000);
	msleep(VSYNC_WAIT);

	display_rgbif_off();
	
	hvga_reconfigure_channel();

	/* Enable tearing fix on CLI if flip closed */
	rodem_set_vdelay(get_legal_refresh_mode(), true);

	reg_write(0x0002, 0x0011);
	rodem_set_rgb_timings(refresh_mode_to_hvga_mode());
	reg_write(0x0010, 0x1100);
	reg_write(0x0002, 0x0211);
	
	rodem_set_display_source(mxcfb_global_state.panel_state);
	rodem_set_panel_mode(get_legal_refresh_mode());
	msleep(1);
	rodem_set_vram_capture_settings(get_legal_refresh_mode()); 
	
	tango_reg_write(0x30, 0x03);
	msleep(VSYNC_WAIT);
}

static void switch_mode2dash_to_mode5dash(refresh_mode_t refresh)
{
	mxcfb_global_state.refresh_mode = refresh;
	switch_mode23dash_to_mode5dash(CLI_PANEL);
}
static void switch_mode3dash_to_mode5dash(refresh_mode_t refresh)
{
	mxcfb_global_state.refresh_mode = refresh;
	switch_mode23dash_to_mode5dash(MAIN_PANEL);
}

static void switch_mode5dash_to_mode2dash(refresh_mode_t refresh)
{
	mxcfb_global_state.refresh_mode = refresh;
	switch_mode5dash_to_mode23dash(CLI_PANEL);
}

static void switch_mode5dash_to_mode3dash(refresh_mode_t refresh) 
{
	mxcfb_global_state.refresh_mode = refresh;
	switch_mode5dash_to_mode23dash(MAIN_PANEL);
}

static void turn_panel_off(void)
{
	stop_memory_capture();
	display_init(tango_off_sequence);
	display_init(rodem_off_sequence);
	hvga_reconfigure_channel();
	gpio_lcd_serializer_stby(ASSERT_GPIO_SIGNAL);
	gpio_lcd_serializer_reset(ASSERT_GPIO_SIGNAL);
	msleep(10);
}

static void sharp_init_panel(void)
{
	msleep(10);
	turn_on_mode5dash();
}

static void sharp_prepare_dma_stop_initialize(void)
{
	stop_memory_capture();
}

static void sharp_prepare_dma_stop_finalize(void)
{
	reg_write(0x0346,0x320); /* VDELAY=800: ignore the external VSYNC inputs */
}

/* NOTE: this function may be run in interrupt context; do not use *sleep */
static void sharp_prepare_dma_start(void)
{
	reg_write(0x0346, 0x00F0);
	rodem_set_vram_capture_settings(get_legal_refresh_mode());
}


/* This command resets 0x007h to the default state */
static void sharp_set_color_depth(color_depth_t color_depth)
{
	switch (color_depth) {
		case low_power_color:
		case low_power_monochrome:
			{
				reg_write(0x402, reg_read(0x402) | (1 << 2) | (1 << 10));
				tango_reg_write(0xEF, 0x00);
				tango_reg_write(0x30, tango_reg_read(0x30) | 0x4);
				tango_reg_write(0xEF, 0x01);
				tango_reg_write(0x30, tango_reg_read(0x30) | 0x4);
				break;
			}
		case full_color:
			{
				reg_write(0x402, reg_read(0x402) & ~(1 << 2) & ~(1 << 10));
				tango_reg_write(0xEF, 0x00);
				tango_reg_write(0x30, tango_reg_read(0x30) & ~0x4);
				tango_reg_write(0xEF, 0x01);
				tango_reg_write(0x30, tango_reg_read(0x30) & ~0x4);
				break;
			}
		default:
			printk("MXCFB ERROR: Invalid color mode passed to %s\n", __func__);
			BUG();
			break;
	}
	tango_assert_valtran();
}

/* Partial Mode timings struct */
struct pm_timings_t {
	uint32_t start_line;
	uint32_t end_line;
	uint32_t ptvw;
	uint32_t ptvf;
	uint32_t ptvm;
	uint32_t ptvb;
	bool disp2src;
};

static int calculate_partial_values(struct pm_timings_t * p)
{
	const uint32_t ptvp = 1;

	/* Ensure valid values */
	if (p->start_line < 0 || p->end_line < 0 || p->start_line > 319 || p->end_line > 319 || p->start_line >= p->end_line) {
		return -EINVAL;
	}
	p->ptvw = (p->end_line - p->start_line);

	/* case1 : start_line + 4 > ptvw + 1 (set DISP2SRC_ON=1) */
	/* case2 : start_line + 4 =< ptvw + 1 (set DISP1SRC_ON=1) */
	if (p->start_line + 4 > p->ptvw + 1) {
		p->disp2src = true;
		p->ptvb = 1;
		p->ptvm = p->start_line + 4 - p->ptvw - 1;
	} else {
		p->disp2src = false;
		p->ptvb = p->start_line + 4;
		p->ptvm = 1;
	}

	/* case3 : 325 > ptvp + ptvb + ptvm + ptvw + ptvw */
	/* case4 : 325 =< ptvp + ptvb + ptvm + ptvw + ptvw */

	if (325 > ptvp + p->ptvb + p->ptvm + p->ptvw + p->ptvw) {
		p->ptvf = 325 - ptvp - p->ptvb - p->ptvm - p->ptvw - p->ptvw;
	} else {
		p->ptvf = 1;
	}

	return 0;
}


static void sharp_enter_low_power_mode(struct partial_mode_info * pm_info)
{
	/****
	 * partial area : 0 =< start_line < end_line < 320			
	 *
	 * Tango partial setting					
	 * R38h: PTLAS = start_line / 2					
	 * R39h: PTLAE = (end_line + 1) / 2					
	 *
	 * case1 : 	start_line + 4 > ptvw + 1		(set DISP2SRC_ON =1)		
	 * case2 : 	start_line + 4 =< ptvw + 1		(set DISP1SRC_ON=1)		
	 * case3 :	325 > ptvp + ptvb + ptvm + ptvw + ptvw				
	 * case4 :	325 =< ptvp + ptvb + ptvm + ptvw + ptvw				
	 *
	 * case1: ptvb = 1		
	 *      : ptvm = start_line + 4 - ptvw - 1
	 * case2: ptvb = 4 + start_line		
	 *      : ptvm = 1	
	 * case3: ptvf=325 - ptvp -ptvb - ptvm - ptvw*2
	 * case4: ptvf = 1	
	 ****/

	struct pm_timings_t pm;
	uint32_t start_lsb = 0;
	uint32_t end_lsb = 0;

	pm.start_line = pm_info->start_y;   
	pm.end_line = pm_info->end_y; 

	start_lsb = pm.start_line % 2;
	end_lsb =  (pm.end_line + 1) % 2;

	if (calculate_partial_values(&pm) < 0) {
		BUG();
		return;
	}

	reg_write(0x0280, 0x0000);
	tango_reg_write(0xEF, 0x01);
	tango_reg_write(0x30, 0x02);
	msleep(VSYNC_WAIT);
	tango_reg_write(0x30, 0x00);
	reg_write(0x0400, 0x0000);
	msleep(VSYNC_WAIT);
	display_rgbif_off();

	hvga_reconfigure_channel();

	tango_reg_write(0x37, (start_lsb << 3) | (end_lsb << 2));
	tango_reg_write(0x38, pm.start_line / 2);
	tango_reg_write(0x39, (pm.end_line + 1) / 2);

	tango_reg_write(0x3A, 0x00);
	tango_reg_write(0x3B, 0x00);
	tango_reg_write(0x3C, 0x1C);
	tango_reg_write(0x3D, 0x10);
	tango_reg_write(0x3F, 0x00);
	tango_reg_write(0x5E, 0x00);
	tango_reg_write(0x80, 0x1E);
	tango_reg_write(0x82, 0xEF);
	tango_reg_write(0x84, 0x00);
	tango_reg_write(0xB1, 0x27);
	tango_reg_write(0xEF, 0x00);
	tango_reg_write(0x14, 0x02);
	tango_reg_write(0x15, 0x12);
	tango_reg_write(0x16, 0x00);
	tango_reg_write(0xEF, 0x01);
	tango_reg_write(0x14, 0x02);
	tango_reg_write(0x15, 0x12);
	tango_reg_write(0x16, 0xA0);
	tango_reg_write(0x42, 0x40);
	tango_reg_write(0x43, 0x06);
	tango_reg_write(0x44, 0x1C);
	tango_reg_write(0x45, 0x03);
	tango_reg_write(0x46, 0x08);
	tango_reg_write(0x47, 0x08);
	tango_reg_write(0x48, 0x0F);
	tango_reg_write(0x49, 0x11);
	tango_reg_write(0x4A, 0x08);

	reg_write(0x042C, 0x000A);
	reg_write(0x042E, 0x0008);
	reg_write(0x0430, 0x00F0);
	reg_write(0x0432, 0x0032);
	reg_write(0x0434, 0x0001);
	reg_write(0x0436, pm.ptvb);
	reg_write(0x0438, pm.ptvw);
	reg_write(0x043C, pm.ptvm);
	reg_write(0x043A, pm.ptvf);

	reg_write(0x0402, 0x0400);
	reg_write(0x0002, 0x0011);
	reg_write(0x000C, 0x0700);
	msleep(10);
	reg_write(0x000E, 0x0333);
	reg_write(0x0010, 0x0300);
	msleep(10);

	reg_write(0x0002, 0x0211);

	rodem_set_ptvlim(true);
	reg_write(0x0458, 0x5001);
	reg_write(0x045A, 0x200A);
	reg_write(0x0364, 0x0000);

	if (pm.disp2src == true) {
		reg_write(0x0400, 0x0121);
	} else {
		reg_write(0x0400, 0x0105);
	}

	reg_write(0x0282, 0x0001);

	tango_reg_write(0x30, 0x0F);

	msleep(VSYNC_WAIT);
	reg_write(0x0002, 0x0011);
	reg_write(0x000E, 0x0233);
	reg_write(0x0010, 0x0500);
	reg_write(0x0002, 0x0211);
	reg_write(0x0040, 0x0002);

	mxcfb_update_spi_frame();
}

static void sharp_update_low_power_mode(struct partial_mode_info * pm_info)
{
	struct pm_timings_t pm;
	uint32_t start_lsb = 0;
	uint32_t end_lsb = 0;

	pm.start_line = pm_info->start_y;   
	pm.end_line = pm_info->end_y; 

	start_lsb = pm.start_line % 2;
	end_lsb =  (pm.end_line + 1) % 2;

	if (calculate_partial_values(&pm) < 0) {
		BUG();
		return;
	}

	reg_write(0x0002, 0x0011);
	reg_write(0x000E, 0x0333);
	reg_write(0x0010, 0x0300);
	msleep(10);
	reg_write(0x0002, 0x0211);
	
	reg_write(0x0400, 0x0000);
	msleep(VSYNC_WAIT);
	
	reg_write(0x0002, 0x0011);
	reg_write(0x0436, pm.ptvb);
	reg_write(0x0438, pm.ptvw);
	reg_write(0x043C, pm.ptvm);
	reg_write(0x043A, pm.ptvf);
	reg_write(0x0002, 0x0211);

	tango_reg_write(0xEF, 0x01);
	tango_reg_write(0x37, (start_lsb << 3) | (end_lsb << 2));
	tango_reg_write(0x38, pm.start_line / 2);
	tango_reg_write(0x39, (pm.end_line + 1) / 2);
	tango_reg_write(0x31, 0x01);

	if (pm.disp2src == true) {
		reg_write(0x0400, 0x0121);
	} else {
		reg_write(0x0400, 0x0105);
	}
	msleep(VSYNC_WAIT);
	reg_write(0x0002, 0x0011);
	reg_write(0x000E, 0x0233);
	reg_write(0x0010, 0x0500);
	reg_write(0x0002, 0x0211);

	mxcfb_update_spi_frame();
}


static void sharp_exit_low_power_mode(void)
{
	reg_write(0x0040, 0x0003);
	reg_write(0x0002, 0x0011);
	rodem_set_clkconf(refresh_mode_to_hvga_mode());
	msleep(10);
	reg_write(0x000E, 0x0333);
	rodem_set_clkdiv2(refresh_mode_to_hvga_mode());
	msleep(10);
	reg_write(0x0002, 0x0211);
	tango_reg_write(0xEF, 0x01);
	tango_reg_write(0xB1, 0x27);
	tango_reg_write(0x30, 0x02);
	msleep(VSYNC_WAIT);
	tango_reg_write(0x30, 0x00);
	reg_write(0x0400, 0x0000);
	msleep(VSYNC_WAIT);

	reg_write(0x0402, 0x0000);
	reg_write(0x0002, 0x0011);
	rodem_set_rgb_timings(refresh_mode_to_hvga_mode());

	display_init(rodem_panel_generator_timings);

	reg_write(0x0002, 0x0211);
	rodem_set_ptvlim(true);

	reg_write(0x0458, 0x5001);
	reg_write(0x045A, 0x200A);
	rodem_set_vdelay(get_legal_refresh_mode(), true);

	tango_reg_write(0xEF, 0x01);
	tango_reg_write(0x3C, 0x00);
	tango_reg_write(0x3D, 0x00);
	tango_reg_write(0x3F, 0x00);
	tango_reg_write(0x5E, 0x08);
	tango_reg_write(0x80, 0x1F);
	tango_reg_write(0x82, 0x81);
	tango_reg_write(0x84, 0x03);
	tango_reg_write(0xEF, 0x00);
	tango_reg_write(0x14, 0x04);
	tango_reg_write(0x15, 0x5A);
	tango_reg_write(0x16, 0xA0);
	tango_reg_write(0xEF, 0x01);
	tango_reg_write(0x14, 0x02);
	tango_reg_write(0x15, 0x5A);
	tango_reg_write(0x16, 0xA0);
	tango_reg_write(0x42, 0x64);
	tango_reg_write(0x43, 0x18);
	tango_reg_write(0x44, 0x50);
	tango_reg_write(0x45, 0x04);
	tango_reg_write(0x46, 0x14);
	tango_reg_write(0x47, 0x14);
	tango_reg_write(0x48, 0x2D);
	tango_reg_write(0x49, 0x36);
	tango_reg_write(0x4A, 0x14);

	rodem_set_display_source(mxcfb_global_state.panel_state);
	rodem_set_panel_mode(get_legal_refresh_mode());
	/* Sleep for 1ms before starting memory capture to avoid flicker */
	msleep(1);
	rodem_set_vram_capture_settings(get_legal_refresh_mode()); 

	hvga_reconfigure_channel();

	tango_reg_write(0x30, 0x03);
	msleep(VSYNC_WAIT);
}

static void sharp_switch_panel_state(panel_t new_state, refresh_mode_t refresh)
{
	panel_t old_state = mxcfb_global_state.panel_state; 
	mxcfb_global_state.panel_state = new_state;

	if (old_state == new_state) {
		printk("MXCFB Warning: old_state = %u, new_state = %u\n", 
				old_state, new_state);
	}

	switch (new_state)
	{
		case PANEL_OFF:
			mxcfb_global_state.refresh_mode = refresh;
			turn_panel_off();
			break;
		case MAIN_PANEL:
			if (old_state == (MAIN_PANEL | CLI_PANEL)) {
				switch_mode5dash_to_mode3dash(refresh);
			} else {
				mxcfb_global_state.refresh_mode = refresh;
				turn_on_mode3dash();
			}
			break;
		case CLI_PANEL:
			if (old_state == (MAIN_PANEL | CLI_PANEL)) {
				switch_mode5dash_to_mode2dash(refresh);
			} else {
				mxcfb_global_state.refresh_mode = refresh;
				turn_on_mode2dash();
			}
			break;
		case MAIN_PANEL | CLI_PANEL:
			if (old_state == MAIN_PANEL) {
				switch_mode3dash_to_mode5dash(refresh);
			} else if (old_state == CLI_PANEL) {
				switch_mode2dash_to_mode5dash(refresh);
			} else {
				mxcfb_global_state.refresh_mode = refresh;
				turn_on_mode5dash();
			}
			break;
		default:
			BUG();
	}
}

static void sharp_rotate_image(image_rotate_t main, image_rotate_t cli)
{
	_sharp_rotate_image(true, main, cli);
}

static void sharp_swap_buffers(void)
{
	uint32_t reg;
	uint32_t disp1, disp2;

	reg = reg_read(0x400);
	disp1 = (reg >> 1) & 0x7;
	disp2 = (reg >> 4) & 0x7;

	reg = (reg & ~(0x7 << 1)) | (disp2 << 1);
	reg = (reg & ~(0x7 << 4)) | (disp1 << 4);
	reg |= (1 << 8);
	reg_write(0x0400, reg);
}

static void sharp_set_refresh_mode(refresh_mode_t refresh)
{
	mxcfb_global_state.refresh_mode = refresh;

	stop_memory_capture();
	msleep(VSYNC_WAIT);

	if (get_legal_refresh_mode() == REFRESH_OFF) {
		/* 
		 * Both panels in static mode
		 */
		display_rgbif_off();
		msleep(VSYNC_WAIT);
		hvga_reconfigure_channel();
	} else {
		/*
		 * Exit static mode - FULL refresh on both main and CLI
		 */
		reg_write(0x045A,0x5001); /* VBLKE VBLK bit goes 1 */
		reg_write(0x0458,0x1001); /* VBLKS VBLK bit goes 0 */

		/* 
		 * If we attempt to disable DMA during the critical 10ms section below, we 
		 * occasionally experience a brief black screen before the screen recovers.
		 * Instead, we should disable DMA here to avoid waiting for the IPU vsync 
		 * below (since hvga_reconfigure_channel calls hvga_disable_channel).
		 */
		hvga_disable_channel(); 
		reg_write(0x0400, 0x0000); /* DISPLAY OFF */
		/****************************************************************************/
		/****** The below section should take less than 10ms to avoid blinking ******/
		/****************************************************************************/
		reg_write(0x0340, 0x0000); /* RGBENA disable */

		hvga_reconfigure_channel();
		rodem_set_rgb_timings(refresh_mode_to_hvga_mode());
		rodem_set_clkdiv2(refresh_mode_to_hvga_mode());
		/****************************************************************************/
		/****** The above section should take less than 10ms to avoid blinking ******/
		/****************************************************************************/
		while((reg_read(0x400)&(0x0200))==0); /* Wait until VBLK=1 (display off) */
		rodem_set_vdelay(get_legal_refresh_mode(), true);
		rodem_set_panel_mode(get_legal_refresh_mode());  /* RGBENA ON */
		rodem_set_display_source(mxcfb_global_state.panel_state);  /* DISPLAY ON */
		rodem_set_ptvlim(true); /* TVLIM=511 */

		reg_write(0x045A, 0x200A); /* VBLKE=200A (Image area line 10) */

		reg_write(0x0458, 0x5001); /* VBLKS=5001 (Frontporch line 1) */
		rodem_set_vram_capture_settings(get_legal_refresh_mode()); 
		msleep(VSYNC_WAIT);
	}
}


/* Global panel information pointers */
struct panel_specific_functions sharp_panel_functions = {
	.disp_conf = &sharp_disp_conf,
	.hvga_panel_info = &sharp_hvga_panel,
	.qvga_panel_info = &sharp_qvga_panel,
	.init_panel = sharp_init_panel,
	.init_channel_template = sharp_init_channel_template,
	.switch_panel_state = sharp_switch_panel_state,
	.enter_low_power_mode = sharp_enter_low_power_mode,
	.update_low_power_mode = sharp_update_low_power_mode,
	.exit_low_power_mode = sharp_exit_low_power_mode,
	.rotate_image = sharp_rotate_image,
	.swap_buffers = sharp_swap_buffers,
	.set_color_depth = sharp_set_color_depth,
	.set_refresh_mode = sharp_set_refresh_mode,
	.prepare_dma_start = sharp_prepare_dma_start,
	.prepare_dma_stop_initialize = sharp_prepare_dma_stop_initialize,
	.prepare_dma_stop_finalize = sharp_prepare_dma_stop_finalize,
	.custom_command = sharp_custom_command,
};

