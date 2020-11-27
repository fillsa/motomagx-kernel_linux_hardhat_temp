/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2006 Motorola, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * DATE          AUTHOR         COMMMENT
 * ----          ------         --------
 * 10/04/2006    Motorola       Added function prototype for mxc_dma_reset
 */

#ifndef SDMA_H
#define SDMA_H

/*!
 * @defgroup SDMA Smart Direct Memory Access (SDMA) Driver
 */

/*!
 * @file sdma.h
 *
 * @brief This file contains the SDMA API declarations.
 *
 * SDMA is responsible on moving data between peripherals and memories (MCU, EMI and DSP).
 *
 * @ingroup SDMA
 */

#include <linux/config.h>
#include <asm/dma.h>
#include <stdarg.h>

#include <asm/hardware.h>

/*!
 * This defines maximum DMA address
 */
#define MAX_DMA_ADDRESS 0xffffffff

/*!
 * This defines maximum number of DMA channels
 */
#ifdef CONFIG_MXC_SDMA_API
#define MAX_DMA_CHANNELS 32
#define MAX_BD_NUMBER    16
#else
#define MAX_DMA_CHANNELS 0
#endif

/*!
 * This enumerates  transfer types
 */
typedef enum {
	emi_2_per = 0,		/*!< EMI memory to peripheral */
	emi_2_int,		/*!< EMI memory to internal RAM */
	emi_2_emi,		/*!< EMI memory to EMI memory */
	emi_2_dsp,		/*!< EMI memory to DSP memory */
	per_2_int,		/*!< Peripheral to internal RAM */
	per_2_emi,		/*!< Peripheral to internal EMI memory */
	per_2_dsp,		/*!< Peripheral to DSP memory */
	int_2_per,		/*!< Internal RAM to peripheral */
	int_2_int,		/*!< Internal RAM to Internal RAM */
	int_2_emi,		/*!< Internal RAM to EMI memory */
	int_2_dsp,		/*!< Internal RAM to DSP memory */
	dsp_2_per,		/*!< DSP memory to peripheral */
	dsp_2_int,		/*!< DSP memory to internal RAM */
	dsp_2_emi,		/*!< DSP memory to EMI memory */
	dsp_2_dsp,		/*!< DSP memory to DSP memory */
	emi_2_dsp_loop,		/*!< EMI memory to DSP memory loopback */
	dsp_2_emi_loop,		/*!< DSP memory to EMI memory loopback */
	dvfs_pll,		/*!< DVFS script with PLL change       */
	dvfs_pdr		/*!< DVFS script without PLL change    */
} sdma_transferT;

/*!
 * This enumerates peripheral types
 */
typedef enum {
	SSI,			/*!< MCU domain SSI */
	SSI_SP,			/*!< Shared SSI */
	MMC,			/*!< MMC */
	SDHC,			/*!< SDHC */
	UART,			/*!< MCU domain UART */
	UART_SP,		/*!< Shared UART */
	FIRI,			/*!< FIRI */
	CSPI,			/*!< MCU domain CSPI */
	CSPI_SP,		/*!< Shared CSPI */
	SIM,			/*!< SIM */
	ATA,			/*!< ATA */
	CCM,			/*!< CCM */
	EXT,			/*!< External peripheral */
	MSHC,			/*!< Memory Stick Host Controller */
	DSP,			/*!< DSP */
	MEMORY			/*!< Memory */
} sdma_periphT;

#ifndef TRANSFER_32BIT
/*!
 * This defines SDMA access data size
 */
#define TRANSFER_32BIT      0x00
#define TRANSFER_8BIT       0x01
#define TRANSFER_16BIT      0x02
#define TRANSFER_24BIT      0x03

#endif

/*!
 * This defines maximum device name length passed during mxc_request_dma().
 */
#define MAX_DEVNAME_LENGTH 32

/*!
 * This defines SDMA interrupt callback function prototype.
 */
typedef void (*dma_callback_t) (void *arg);

/*!
 * Structure containing sdma channel parameters.
 */
typedef struct {
	__u32 watermark_level;	/*!< Lower/upper threshold that
				 *   triggers SDMA event
				 */
	__u32 per_address;	/*!< Peripheral source/destination
				 *   physical address
				 */
	sdma_periphT peripheral_type;	/*!< Peripheral type */
	sdma_transferT transfer_type;	/*!< Transfer type   */
	int event_id;		/*!< Event number,
				 *   needed by all channels
				 *   that started by peripherals dma
				 *   request (per_2_*,*_2_per)
				 *   Not used for memory and DSP
				 *   transfers.
				 */
	int event_id2;		/*!< Second event number,
				 *   used in ATA scripts only.
				 */
	int bd_number;		/*!< Buffer descriptors number.
				 *   If not set, single buffer
				 *   descriptor will be used.
				 */
	dma_callback_t callback;	/*!   callback function            */
	void *arg;		/*!   callback argument            */
	unsigned long word_size:8;	/*!< SDMA data access word size    */
} dma_channel_params;

/*!
 * Structure containing sdma request  parameters.
 */
typedef struct {
	/*!   physical source memory address        */
	__u8 *sourceAddr;
	/*!   physical destination memory address   */
	__u8 *destAddr;
	/*!   amount of data to transfer,
	 * updated during mxc_dma_get_config
	 */
	__u16 count;
	/*!< DONE bit of the buffer descriptor,
	 * updated during mxc_dma_get_config
	 * 0 - means the BD is done and closed by SDMA
	 * 1 - means the BD is still being processed by SDMA
	 */
	int bd_done;
	/*!< CONT bit of the buffer descriptor,
	 * set it if full multi-buffer descriptor mechanism
	 * required.
	 */
	int bd_cont;
	/*!< ERROR bit of the buffer descriptor,
	 * updated during mxc_dma_get_config.
	 * If it is set - there was an error during BD processing.
	 */
	int bd_error;
} dma_request_t;

/*!
 * Structure containing sdma request  parameters.
 */
typedef struct {
	/*! address of ap_2_ap script */
	int mxc_sdma_ap_2_ap_addr;
	/*! address of ap_2_bp script */
	int mxc_sdma_ap_2_bp_addr;
	/*! address of bp_2_ap script */
	int mxc_sdma_bp_2_ap_addr;
	/*! address of loopback_on_dsp_side script */
	int mxc_sdma_loopback_on_dsp_side_addr;
	/*! address of mcu_interrupt_only script */
	int mxc_sdma_mcu_interrupt_only_addr;

	/*! address of firi_2_per script */
	int mxc_sdma_firi_2_per_addr;
	/*! address of firi_2_mcu script */
	int mxc_sdma_firi_2_mcu_addr;
	/*! address of per_2_firi script */
	int mxc_sdma_per_2_firi_addr;
	/*! address of mcu_2_firi script */
	int mxc_sdma_mcu_2_firi_addr;

	/*! address of uart_2_per script */
	int mxc_sdma_uart_2_per_addr;
	/*! address of uart_2_mcu script */
	int mxc_sdma_uart_2_mcu_addr;
	/*! address of per_2_app script */
	int mxc_sdma_per_2_app_addr;
	/*! address of mcu_2_app script */
	int mxc_sdma_mcu_2_app_addr;

	/*! address of uartsh_2_per script */
	int mxc_sdma_uartsh_2_per_addr;
	/*! address of uartsh_2_mcu script */
	int mxc_sdma_uartsh_2_mcu_addr;
	/*! address of per_2_shp script */
	int mxc_sdma_per_2_shp_addr;
	/*! address of mcu_2_shp script */
	int mxc_sdma_mcu_2_shp_addr;

	/*! address of ata_2_mcu script */
	int mxc_sdma_ata_2_mcu_addr;
	/*! address of mcu_2_ata script */
	int mxc_sdma_mcu_2_ata_addr;

	/*! address of app_2_per script */
	int mxc_sdma_app_2_per_addr;
	/*! address of app_2_mcu script */
	int mxc_sdma_app_2_mcu_addr;
	/*! address of shp_2_per script */
	int mxc_sdma_shp_2_per_addr;
	/*! address of shp_2_mcu script */
	int mxc_sdma_shp_2_mcu_addr;

	/*! address of mshc_2_mcu script */
	int mxc_sdma_mshc_2_mcu_addr;
	/*! address of mcu_2_mshc script */
	int mxc_sdma_mcu_2_mshc_addr;

	/*! address of dptc_dvfs script */
	int mxc_sdma_dptc_dvfs_addr;

	/*! address where ram code starts */
	int mxc_sdma_ram_code_start_addr;
	/*! size of the ram code */
	int mxc_sdma_ram_code_size;
	/*! RAM image address */
	unsigned short *mxc_sdma_start_addr;
} sdma_script_start_addrs;

/*!
 * Setup channel according to parameters.
 * Must be called once after mxc_request_dma()
 *
 * @param   channel           channel number
 * @param   p                 channel parameters pointer
 * @return  0 on success, error code on fail
 */
int mxc_dma_setup_channel(int channel, dma_channel_params * p);

/*!
 * Allocates dma channel.
 * If channel's value is 0, then the function allocates a free channel
 * dynamically and sets its value to channel.
 * Else allocates requested channel if it is free.
 * If the channel is busy or no free channels (in dynamic allocation) -EBUSY returned.
 *
 * @param   channel           pointer to channel number
 * @param   devicename        device name
 * @return  0 on success, error code on fail
 */
int mxc_request_dma(int *channel, const char *devicename);

/*!
 * Configures request parameters. Can be called multiple times after
 * mxc_request_dma() and mxc_dma_setup_channel().
 *
 *
 * @param   channel           channel number
 * @param   p                 request parameters pointer
 * @param   bd_index          index of buffer descriptor to set
 * @return  0 on success, error code on fail
 */
/* int mxc_dma_set_config(int channel, dma_request_t *p, int bd_index); */
int mxc_dma_set_config(int channel, dma_request_t * p, int bd_index);

/*!
 * Returns request parameters.
 *
 * @param   channel           channel number
 * @param   p                 request parameters pointer
 * @param   bd_index          index of buffer descriptor to get
 * @return  0 on success, error code on fail
 */
/* int mxc_dma_get_config(int channel, dma_request_t *p, int bd_index); */
int mxc_dma_get_config(int channel, dma_request_t * p, int bd_index);

/*!
 * Starts dma channel.
 *
 * @param   channel           channel number
 */
int mxc_dma_start(int channel);

#ifdef CONFIG_MOT_WFN409
/*!
 * Stop the current transfer
 *
 * @param   channel           channel number
 * @param   buffer_number     number of buffers (beginning with 0),
 *                            whose done bits should be reset to 0
*/
int mxc_dma_reset(int channel, int buffer_number);
#endif /* CONFIG_MOT_WFN409 */

/*!
 * Stops dma channel.
 *
 * @param   channel           channel number
 */
int mxc_dma_stop(int channel);

/*!
 * Frees dma channel.
 *
 * @param   channel           channel number
 */
void mxc_free_dma(int channel);

/*!
 * Sets callback function. Used with standard dma api
 *  for supporting interrupts
 *
 * @param   channel           channel number
 * @param   callback          callback function pointer
 * @param   arg               argument for callback function
 */
void mxc_dma_set_callback(int channel, dma_callback_t callback, void *arg);

/*!
 * Allocates uncachable buffer. Uses hash table.
 *
 * @param   size    size of allocated buffer
 * @return  pointer to buffer
 */
void *sdma_malloc(size_t size);

/*!
 * Frees uncachable buffer. Uses hash table.
 */
void sdma_free(void *buf);

/*!
 * Converts virtual to physical address. Uses hash table.
 *
 * @param   buf  virtual address pointer
 * @return  physical address value
 */
unsigned long sdma_virt_to_phys(void *buf);

/*!
 * Converts physical to virtual address. Uses hash table.
 *
 * @param   buf  physical address value
 * @return  virtual address pointer
 */
void *sdma_phys_to_virt(unsigned long buf);

/*!
 * Initializes SDMA driver
 */
int __init sdma_init(void);

#define DEFAULT_ERR     1
#define MXC_SDMA_DSPDMA 1

#endif
