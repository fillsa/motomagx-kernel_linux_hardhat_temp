/*
 *  mxc_camera.h
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

Revision History:
                    Modification    
Author                  Date        Description of Changes
----------------    ------------    -------------------------
Motorola            12/19/2003      Created
Motorola            02/26/2004      Update algorithm for DMA transfer
Motorola            06/22/2005      Make changes to build with
                                    2.6 kernel for SCMA11
Motorola            08/09/2005      Make use of IPU on SCM-A11
Motorola            11/16/2005      EZXBASE 36 upmerge
Motorola            12/09/2005      Fix EZXBASE36 upmerge issues.
Motorola            03/03/2006      EZXBASE 48 upmerge
Motorola            07/12/2006      Viewfinder causing very long response
                                    delay to user input
Motorola            12/07/2006      OSS code changes
Motorola            01/04/2007      OSS code changes
*/

/*================================================================================
                                 INCLUDE FILES
================================================================================*/
#ifndef MXC_CAMERA_H_
#define MXC_CAMERA_H_

#include <linux/camera.h>

#undef DEBUG
 
//-------------------------------------------------------------------------------------------------------
//      Control APIs
//-------------------------------------------------------------------------------------------------------
int  ci_init(void);
void ci_deinit(void);

#define err_print(fmt, args...) printk(KERN_ERR "fun %s "fmt"\n", __FUNCTION__, ##args)

#ifdef DEBUG

#define dbg_print(fmt, args...) printk(KERN_INFO "fun %s "fmt"\n", __FUNCTION__, ##args)

#if DEBUG > 1
#define ddbg_print(fmt, args...) printk(KERN_INFO "fun %s "fmt"\n", __FUNCTION__, ##args)
#else
#define ddbg_print(fmt, args...) ;
#endif

#else

#define dbg_print(fmt, args...)  ;
#define ddbg_print(fmt, args...) ;

#endif

/*
Structures
*/
typedef struct camera_context_s camera_context_t, *p_camera_context_t;

// Capture status
#define CAMERA_STATUS_VIDEO_CAPTURE_IN_PROCESS 0x0001
#define CAMERA_STATUS_RING_BUFFER_FULL         0x0002

#define CAMERA_ERROR_NONE                      0
#define CAMERA_ERROR_UNEXPECTEDINT             -1

typedef struct {
    int (*init)(p_camera_context_t context);
    int (*deinit)(p_camera_context_t);   
    int (*set_capture_format)(p_camera_context_t);
    int (*start_capture)(p_camera_context_t, unsigned int frames);
    int (*stop_capture)(p_camera_context_t);
    int (*command)(p_camera_context_t, unsigned int cmd, void *param);
    int (*pm_management)(p_camera_context_t, int suspend);
} camera_function_t, *p_camera_function_t;
// context

struct camera_context_s {
	// syncronization stuff
	atomic_t refcount;

	/*
	 * DRIVER FILLED PARAMTER
	 */

	// sensor info  
	unsigned int sensor_type;

	// capture image info
	unsigned int capture_width; 
	unsigned int capture_height;
    unsigned int sensor_width;
    unsigned int sensor_height;
    unsigned int still_width;
    unsigned int still_height;

	unsigned int    capture_input_format;
	unsigned int    capture_output_format;
    unsigned int    still_input_format;
    unsigned int    still_output_format;

    unsigned int    capture_digital_zoom;
    unsigned int    still_digital_zoom;

    int             plane_number;

    int             still_image_mode;

    int             jpeg_quality;

    int             waiting_frame;
    int             fv_rising_edge;
    int             preferred_block_num;
    int             still_image_rdy;
    int             task_waiting;
    int             detected_sensor_type;
    int             vf_configured;

	V4l_PIC_STYLE     capture_style;
	V4l_PIC_WB        capture_light;
	int             capture_bright;
	int             flicker_freq;

	// general information
	char            name[32];
	min_max_t       capability;
    
	// frame rate control
    unsigned int frame_rate;
	unsigned int fps;
    unsigned int mini_fps;
    
    unsigned int mclk;

   	// ring buffers
	// note: must pass in 8 bytes aligned address
	void *buffer_virtual;
	void *buffer_physical;
	unsigned int buf_size;

	// memory for dma descriptors, layout:
	//  dma descriptor chain 0,
	//  dma descriptor chain 1,
	//  ...  
	void *dma_descriptors_virtual;
	void *dma_descriptors_physical;
	unsigned int dma_descriptors_size;

	// os mapped register address   
	unsigned int clk_reg_base;
	unsigned int ost_reg_base;
	unsigned int gpio_reg_base;
	unsigned int ci_reg_base;
	unsigned int board_reg_base;

	// function dispatch table
	p_camera_function_t camera_functions;

	/*
	 * FILLED PARAMTER
	 */
	int dma_channels[3];
	unsigned int capture_status;

    unsigned planeOffset[MAX_FRAMES][3];
    unsigned planeBytes[MAX_FRAMES][3];

	/*
	 * INTERNALLY USED: DON'T TOUCH!
	 */
	unsigned int block_number, block_size, block_number_max;
	unsigned int block_header, block_tail;
	unsigned int fifo0_descriptors_virtual, fifo0_descriptors_physical;
	unsigned int fifo1_descriptors_virtual, fifo1_descriptors_physical;
	unsigned int fifo2_descriptors_virtual, fifo2_descriptors_physical;
	unsigned int fifo0_num_descriptors;
	unsigned int fifo1_num_descriptors;
	unsigned int fifo2_num_descriptors;
	unsigned int fifo0_transfer_size;
	unsigned int fifo1_transfer_size;
	unsigned int fifo2_transfer_size;

	struct page **page_array;

	unsigned int pages_allocated;
	unsigned int page_aligned_block_size;
	unsigned int pages_per_block;
	unsigned int pages_per_fifo0;
	unsigned int pages_per_fifo1;
	unsigned int pages_per_fifo2;

#ifdef CONFIG_DPM
        struct pm_dev *pmdev;
#endif
	int dma_started;
};

/*
Prototypes
*/
/***********************************************************************
 *
 * Capture APIs
 *
 ***********************************************************************/
// skip frame before capture video or still image
void camera_skip_frame( p_camera_context_t camera_context, int skip_frame_num );


/***********************************************************************
 *
 * Interrupt APIs
 *
 ***********************************************************************/
// gpio init
void camera_gpio_init(void);
void camera_gpio_deinit(void);

// vcam control
void camera_turn_on_vcam(void);
void camera_turn_off_vcam(void);

#endif
