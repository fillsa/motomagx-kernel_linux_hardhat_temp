/*
 * Copyright 2004-2005 Freescale Semiconductor, Inc.
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
 * 10/2006  Motorola  Initial version. Added panel configuration structs for
 *                    initializing HVGA displays.
 * 11/2006  Motorola  Updated panel_specific_functions with set_low_power_mode
 * 02/2007  Motorola  Refactored driver to support multiple displays
 * 03/2007  Motorola  Changed refresh rate from 30Hz to 50Hz
 * 04/2007  Motorola  Added rotate_image callback handler
 * 04/2007  Motorola  Added ADC template and deserializer reset callback functions
 * 06/2007  Motorola  Fixed issue with ESD routine occuring outside of vsync
 * 06/2007  Motorola  Updated prepare_dma_start callback handler
 */

#ifndef _MXCFB_HVGA_H
#define _MXCFB_HVGA_H

#include "mxcfb.h"
#include <linux/motfb.h>

/* RGB refresh rate to the panel */
#define REFRESH_RATE 50 
/* Panel Vsync wait period in ms */
#define VSYNC_WAIT ((1000 / REFRESH_RATE) + 1)

#define PANEL_ON 1
#define PANEL_OFF 0
#define HVGA_MODE 1
#define QVGA_MODE 0

/* Extern global state on mxcfb_hvga file */
extern struct global_state mxcfb_global_state;

/* ADC display configuration settings */
struct adc_display_conf {
	/* Serial timing values for write accesses (in nanoseconds) */
	uint32_t write_cycle_time;
	uint32_t write_up_time;
	uint32_t write_down_time;

	/* Serial timing values for read accesses (in nanoseconds) */
	uint32_t read_cycle_time;
	uint32_t read_up_time;
	uint32_t read_down_time;
	uint32_t read_latch_time;
	uint32_t read_pixel_clock;

	/* Interface settings for ADC displays */
	uint32_t pix_fmt;  /* Pixel packing format for the Display interface */
	display_addressing_t addressing_mode;  /* Full or XY addressing mode */
	uint32_t vsync_width;
	uint32_t vsync_mode;

	/* Duplicated from ipu_adc_sig_cfg_t */
	ipu_adc_sig_cfg_t sig;
};

/* Panel initialization command table */
struct panel_init_commands {
	uint32_t index;
	uint32_t data;
	cmddata_t mode;
	uint32_t msleep;
};

static inline int mxcfb_get_disp_num(void)
{
#if defined(CONFIG_MACH_ASCENSION) || defined(CONFIG_MACH_LIDO) || \
	defined(CONFIG_MACH_SAIPAN)
	return DISP1;
#elif defined(CONFIG_MACH_SCMA11REF)
	return DISP2;
#else
#error ERROR: Display number not defined for machine type
#endif
}

/* Panel specific information and function pointers */
struct panel_specific_functions
{
	const struct adc_display_conf * disp_conf;
	const struct panel_info * hvga_panel_info;
	const struct panel_info * qvga_panel_info;
	void (*init_panel)(void);
	void (*init_channel_template)(void);
	void (*switch_panel_state)(panel_t, refresh_mode_t);
	void (*enter_low_power_mode)(struct partial_mode_info *);
	void (*update_low_power_mode)(struct partial_mode_info *);
	void (*exit_low_power_mode)(void);
	void (*swap_buffers)(void);
	void (*set_color_depth)(color_depth_t color_depth);
	void (*set_refresh_mode)(refresh_mode_t);
	void (*rotate_image)(image_rotate_t main, image_rotate_t cli);
	void (*prepare_dma_start)(void);
	void (*prepare_dma_stop_initialize)(void);
	void (*prepare_dma_stop_finalize)(void);
	int  (*custom_command)(int disp, const struct panel_init_commands * command);
};

extern void display_init(const struct panel_init_commands * commands);
extern void reg_write(uint32_t index, uint32_t command);
extern uint32_t reg_read(uint32_t index);
extern int poll_read(uint32_t index, uint32_t mask, uint32_t compare);
extern void hvga_disable_channel(void);
extern int hvga_reconfigure_channel(void);
extern refresh_mode_t panel_to_refresh_mask(panel_t panel);
extern refresh_mode_t get_legal_refresh_mode(void);
extern uint32_t panel_state_to_hvga_mode(void);
extern uint32_t refresh_mode_to_hvga_mode(void);
extern int mxcfb_update_spi_frame(void);

/* Extern callbacks on panel specific files */
extern struct panel_specific_functions E02_panel_functions;
extern struct panel_specific_functions sharp_panel_functions;

#endif /* _MXCFB_HVGA_H */
