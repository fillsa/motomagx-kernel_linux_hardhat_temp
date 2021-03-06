/*
 * Copyright 2006 Motorola, Inc.
 *
 * Changelog:
 * Date        Author         Comment
 * 10/05/2006  Motorola       added Freescale product name changes to existing source
 *                            base to allow compilation
*/
                                                                                                                                                          
/*
 * Copyright 2004 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef MXC_FIRI_REG_H

#define MXC_FIRI_REG_H
#include <asm/arch/platform.h>

/*!
 * @defgroup FIRI Fast IR Driver
 */

/*!
 * @file mxc_fir.h
 * 
 * @brief  MXC FIRI header file
 *
 * This file defines base address and bits of FIRI registers 
 * 
 * @ingroup FIRI
 */

/*!
 * Define FIRI_INT based on architecture
 */
#ifdef CONFIG_ARCH_MX3
#define FIRI_INT INT_FIRI
#define UART_BASE_ADDR
#define UART_INT
#else
#ifdef CONFIG_ARCH_MXC91331 
#define FIRI_INT INT_UART4_FIRI_OR
#define UART_BASE_ADDR UART4_BASE_ADDR
#define UART_INT INT_UART4_FIRI_OR
#endif
#ifdef CONFIG_ARCH_MXC91131 
#define FIRI_INT INT_FIRI
#define UART_BASE_ADDR UART2_BASE_ADDR
#define UART_INT INT_FIRI
#endif
#endif

/*!
 * FIRI maximum packet length 
 */
#define FIR_MAX_RXLEN           2047

/*  
 * FIRI Transmitter Control Register 
 */
#define FIRITCR (*((volatile __u32*)(IO_ADDRESS(FIRI_BASE_ADDR + 0x00))))
/*
 * FIRI Transmitter Count Register 
 */
#define FIRITCTR (*((volatile __u32*)(IO_ADDRESS(FIRI_BASE_ADDR + 0x04))))
/*
 * FIRI Receiver Control Register 
 */
#define FIRIRCR (*((volatile __u32*)(IO_ADDRESS(FIRI_BASE_ADDR + 0x08))))
/*
 * FIRI Transmitter Status Register
 */
#define FIRITSR (*((volatile __u32*)(IO_ADDRESS(FIRI_BASE_ADDR + 0x0C))))
/*
 * FIRI Receiver Status Register
 */
#define FIRIRSR (*((volatile __u32*)(IO_ADDRESS(FIRI_BASE_ADDR + 0x10))))
/*
 * FIRI Transmitter FIFO
 */
#define FIRITXFIFO (*((volatile __u32*)(IO_ADDRESS(FIRI_BASE_ADDR + 0x14))))
/*
 * FIRI Receiver FIFO
 */
#define FIRIRXFIFO (*((volatile __u32*)(IO_ADDRESS(FIRI_BASE_ADDR + 0x18))))
/* 
 * FIRI Control Register 
 */
#define FIRICR (*((volatile __u32*)(IO_ADDRESS(FIRI_BASE_ADDR + 0x1C))))

/*
 * Bit definitions of Transmitter Controller Register 
 */
#define FIRITCR_HAG     (1<<24) /* H/W address generator */
#define FIRITCR_SRF_FIR (0<<13) /* Start field repeat factor */
#define FIRITCR_SRF_MIR (1<<13) /* Start field Repeat Factor */
#define FIRITCR_TDT_MIR (2<<10) /* TX trigger for MIR is set to 32 bytes) */
#define FIRITCR_TDT_FIR (1<<10) /* TX trigger for FIR is set to 16 bytes) */
#define FIRITCR_TCIE    (1<<9)  /* TX Complete Interrupt Enable */
#define FIRITCR_TPEIE   (1<<8)  /* TX Packet End Interrupt Enable */
#define FIRITCR_TFUIE   (1<<7)  /* TX FIFO Under-run Interrupt Enable */
#define FIRITCR_PCF     (0<<6)  /* Packet Complete by FIFO */
#define FIRITCR_PC      (0<<5)  /* Packet Complete */
#define FIRITCR_SIP     (0<<4)  /* TX Enable of SIP */
#define FIRITCR_TPP     (0<<3)  /* TX Pulse Polarity bit */
#define FIRITCR_TM_FIR  (0<<1)  /* TX Mode 4 Mbps */
#define FIRITCR_TM_MIR1 (1<<1)  /* TX Mode 0.576 Mbps */
#define FIRITCR_TM_MIR2 (1<<2)  /* TX Mode 1.152 Mbps */
#define FIRITCR_TE      (1<<0)  /* TX Enable */

/*
 * Bit definitions of Transmitter Count Register 
 */
#define FIRITCTR_TPL     511    /* TX Packet Length set to 512 bytes */

/*
 * Bit definitions of Receiver Control Register
 */
#define FIRIRCR_RAM     (0<<24) /* RX Address Match */
#define FIRIRCR_RPEDE   (1<<11) /* Packet End DMA request Enable */
#define FIRIRCR_RDT_MIR (2<<8)  /* DMA Trigger level(64 bytes in RXFIFO) */
#define FIRIRCR_RDT_FIR (1<<8)  /* DMA Trigger level(80 bytes in RXFIFO) */
#define FIRIRCR_RPA     (1<<7)  /* RX Packet Abort */
#define FIRIRCR_RPEIE   (1<<6)  /* RX Packet End Interrupt Enable */
#define FIRIRCR_PAIE    (1<<5)  /* Packet Abort Interrupt Enable */
#define FIRIRCR_RFOIE   (1<<4)  /* RX FIFO Overrun Interrupt Enable */
#define FIRIRCR_RPP     (0<<3)  /* RX Pulse Polarity bit */
#define FIRIRCR_RM_FIR  (0<<1)  /* 4 Mbps */
#define FIRIRCR_RM_MIR1 (1<<1)  /* 0.576 Mbps */
#define FIRIRCR_RM_MIR2 (1<<2)  /* 1.152 Mbps */
#define FIRIRCR_RE      (1<<0)  /* RX Enable */

/* Transmitter Status Register */
#define FIRITSR_TFP     0xFF00  /* Mask for available bytes in TX FIFO */
#define FIRITSR_TC      (1<<3)  /* Transmit Complete bit */
#define FIRITSR_SIPE    (1<<2)  /* SIP End bit */
#define FIRITSR_TPE     (1<<1)  /* Transmit Packet End */
#define FIRITSR_TFU     (1<<0)  /* TX FIFO Under-run */

/* Receiver Status Register */
#define FIRIRSR_RFP     0xFF00  /* mask for available bytes RX FIFO */
#define FIRIRSR_PAS     (1<<5)  /* preamble search */
#define FIRIRSR_RPE     (1<<4)  /* RX Packet End */
#define FIRIRSR_RFO     (1<<3)  /* RX FIFO Overrun */
#define FIRIRSR_BAM     (1<<2)  /* Broadcast Address Match */
#define FIRIRSR_CRCE    (1<<1)  /* CRC error */
#define FIRIRSR_DDE     (1<<0)  /* Address, control or data field error */

/* FIRI Control Register */
#define FIRICR_BL       (32<<5)  /* Burst Length is set to 32*/
#define FIRICR_OSF      (0<<1)   /* Over Sampling Factor */

#endif /* MXC_FIRI_REG_H */
