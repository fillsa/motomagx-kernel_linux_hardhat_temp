/*
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2006 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* Date         Author          Comment
 * ===========  ==============  ==============================================
 * 04-Oct-2006  Motorola        Add preliminary support for Motorola GPIO API.
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include "iomux.h"

#if defined(CONFIG_MOT_FEAT_GPIO_API)
#include <asm/mot-gpio.h>
#endif /* CONFIG_MOT_FEAT_GPIO_API */

/*!
 * @file mxc91131evb_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO
 */

#if defined(CONFIG_MOT_FEAT_GPIO_API)
/**
 * Initial GPIO register settings.
 */
struct gpio_signal_settings initial_gpio_settings[MAX_GPIO_SIGNAL] = {
};
#endif /* CONFIG_MOT_FEAT_GPIO_API */


/*!
 * This system-wise GPIO function initializes the pins during system startup.
 * All the statically linked device drivers should put the proper GPIO initialization
 * code inside this function. It is called by \b fixup_mxc_board() during
 * system startup. This function is board specific.
 */
void mxc91131evb_gpio_init(void)
{
	/*
	 * Configure GPIO C4 as an output driven high.
	 * This is connected to the SC55112 WDI input.
	 */
	gpio_set_data(2, 4, 1);
	gpio_config(2, 4, true, GPIO_INT_NONE);

}

/*!
 * Setup GPIO for a UART port to be active
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_active(int port, int no_irda)
{
	unsigned int pbc_bctrl1_set = 0;

	/*
	 * Configure the IOMUX control registers for the UART signals
	 * and enable the UART transceivers
	 */
	switch (port) {
		/* UART 1 IOMUX Configs */
	case 0:
		iomux_config_mux(U1_TXD_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U1_RXD_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U1_RTS_B_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U1_CTS_B_PIN, MUX0_OUT, MUX0_IN);
		/*
		 * Enable the UART 1 Transceiver
		 */
		pbc_bctrl1_set |= PBC_BCTRL1_UENA;
		break;
		/* UART 2 IOMUX Configs */
	case 1:
		iomux_config_mux(U2_TXD_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U2_RXD_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U2_RTS_B_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U2_CTS_B_PIN, MUX0_OUT, MUX0_IN);
		/*
		 * Enable the UART 2 Transceiver or the Irda Transceiver
		 */
		if (no_irda == 1) {
			pbc_bctrl1_set |= PBC_BCTRL1_UENB;
		} else {
			iomux_config_mux(GP_AP_C1_PIN, GPIO_MUX1_OUT, NONE_IN);
			gpio_config(2, 1, true, GPIO_INT_NONE);
			/* Clear the SD/Mode signal */
			gpio_set_data(2, 1, 0);
			pbc_bctrl1_set |= PBC_BCTRL1_IREN;
		}
		break;
		/* UART 3 IOMUX Configs */
	case 2:
		iomux_config_mux(USB_DAT_VP_PIN, MUX4_OUT, NONE_IN);	// TXD
		iomux_config_mux(USB_SE0_VM_PIN, MUX2_OUT, MUX2_IN);	// RXD
		iomux_config_mux(USB_RXD_PIN, MUX2_OUT, MUX2_IN);	// RTS
		iomux_config_mux(U3CE_CTS_B_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U3CE_DSR_B_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U3CE_RI_B_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U3CE_DCD_B_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(U3CE_DTR_B_PIN, MUX0_OUT, MUX0_IN);
		/*
		 * Enable the UART 3 Transceiver
		 */
		pbc_bctrl1_set |= PBC_BCTRL1_UENCE;
		break;
	default:
		break;
	}
	__raw_writew(pbc_bctrl1_set, PBC_BASE_ADDRESS + PBC_BCTRL1_SET);
	/*
	 * TODO: Configure the Pad registers for the UART pins
	 */
}

/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
	unsigned int pbc_bctrl1_clr = 0;

	/*
	 * Disable the UART Transceiver by configuring the GPIO pin
	 */
	switch (port) {
	case 0:
		/*
		 * Disable the UART 1 Transceiver
		 */
		pbc_bctrl1_clr |= PBC_BCTRL1_UENA;
		break;
	case 1:
		/*
		 * Disable the UART 2 Transceiver
		 */
		pbc_bctrl1_clr |= PBC_BCTRL1_UENB;
		break;
	case 2:
		/*
		 * Disable the UART 3 Transceiver
		 */
		pbc_bctrl1_clr |= PBC_BCTRL1_UENCE;
		break;
	default:
		break;
	}
	__raw_writew(pbc_bctrl1_clr, PBC_BASE_ADDRESS + PBC_BCTRL1_CLEAR);
}

/*
 * The function is stubbed out and does not apply for MXC91131
 */
void config_uartdma_event(int port)
{
	return;
}

/*!
 *  Setup GPIO for keypad to be active
 */
void gpio_keypad_active(void)
{
	iomux_config_mux(KPROW0_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPROW1_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPROW2_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPROW3_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPROW4_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPROW5_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPROW6_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPROW7_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPCOL0_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPCOL1_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPCOL2_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPCOL3_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPCOL4_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPCOL5_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPCOL6_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(KPCOL7_PIN, MUX0_OUT, MUX0_IN);
}

EXPORT_SYMBOL(gpio_keypad_active);

/*!
 * Setup GPIO for a keypad to be inactive
 */
void gpio_keypad_inactive(void)
{
	iomux_config_mux(KPROW0_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPROW1_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPROW2_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPROW3_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPROW4_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPROW5_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPROW6_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPROW7_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPCOL0_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPCOL1_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPCOL2_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPCOL3_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPCOL4_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPCOL5_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPCOL6_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(KPCOL7_PIN, GPIO_MUX1_OUT, NONE_IN);
}

EXPORT_SYMBOL(gpio_keypad_inactive);

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1_SS0 IOMux configuration */
		iomux_config_mux(SPI1_SS0_PIN, MUX0_OUT, MUX0_IN);
		/* SPI1 MISO IOMux configuration */
		iomux_config_mux(SPI1_MISO_PIN, MUX0_OUT, MUX0_IN);
		/* SPI1 MOSI IOMux configuration */
		iomux_config_mux(SPI1_MOSI_PIN, MUX0_OUT, MUX0_IN);
		/* SPI1_SS1 IOMux configuration */
		iomux_config_mux(SPI1_SS1_PIN, MUX0_OUT, MUX0_IN);
		/* SPI1 CLK IOMux configuration */
		iomux_config_mux(SPI1_CLK_PIN, MUX0_OUT, MUX0_IN);
		break;
	case 1:
		/* SPI2_SS0 IOMux configuration */
		iomux_config_mux(SPI2_SS0_PIN, MUX0_OUT, MUX0_IN);
		/* SPI2 MISO IOMux configuration */
		iomux_config_mux(SPI2_MISO_PIN, MUX0_OUT, MUX0_IN);
		/* SPI2 MOSI IOMux configuration */
		iomux_config_mux(SPI2_MOSI_PIN, MUX0_OUT, MUX0_IN);
		/* SPI2_SS1 IOMux configuration */
		iomux_config_mux(SPI2_SS1_PIN, MUX0_OUT, MUX0_IN);
		/* SPI2 CLK IOMux configuration */
		iomux_config_mux(SPI2_CLK_PIN, MUX0_OUT, MUX0_IN);
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1_SS0 IOMux configuration */
		iomux_config_mux(SPI1_SS0_PIN, GPIO_MUX1_OUT, NONE_IN);
		/* SPI1 MISO IOMux configuration */
		iomux_config_mux(SPI1_MISO_PIN, GPIO_MUX1_OUT, NONE_IN);
		/* SPI1 MOSI IOMux configuration */
		iomux_config_mux(SPI1_MOSI_PIN, GPIO_MUX1_OUT, NONE_IN);
		/* SPI1_SS1 IOMux configuration */
		iomux_config_mux(SPI1_SS1_PIN, GPIO_MUX1_OUT, NONE_IN);
		/* SPI1 CLK IOMux configuration */
		iomux_config_mux(SPI1_CLK_PIN, GPIO_MUX1_OUT, NONE_IN);
		break;
	case 1:
		/* SPI2_SS0 IOMux configuration */
		iomux_config_mux(SPI2_SS0_PIN, GPIO_MUX1_OUT, NONE_IN);
		/* SPI2 MISO IOMux configuration */
		iomux_config_mux(SPI2_MISO_PIN, GPIO_MUX1_OUT, NONE_IN);
		/* SPI2 MOSI IOMux configuration */
		iomux_config_mux(SPI2_MOSI_PIN, GPIO_MUX1_OUT, NONE_IN);
		/* SPI2_SS1 IOMux configuration */
		iomux_config_mux(SPI2_SS1_PIN, GPIO_MUX1_OUT, NONE_IN);
		/* SPI2 CLK IOMux configuration */
		iomux_config_mux(SPI2_CLK_PIN, GPIO_MUX1_OUT, NONE_IN);
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		iomux_config_mux(I2CLK_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(I2DAT_PIN, MUX0_OUT, MUX0_IN);
	default:
		break;
	}
}

/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		iomux_config_mux(I2CLK_PIN, GPIO_MUX1_OUT, NONE_IN);
		iomux_config_mux(I2DAT_PIN, GPIO_MUX1_OUT, NONE_IN);
		break;
	default:
		break;
	}
}

/*!
 * This function configures the IOMux block for the SC55112 PMIC
 *
 */
void gpio_sc55112_active(void)
{
	iomux_config_mux(ED_INT1_PIN, MUX0_OUT, MUX0_IN);
}

/*!
 * This function configures the IOMux block for the SC55112 PMIC
 *
 */
void gpio_sc55112_inactive(void *irq_handler)
{
	iomux_config_mux(ED_INT1_PIN, GPIO_MUX1_OUT, NONE_IN);
}

/*!
 * This function configures the IOMux block so that the ED_INT7
 * pin is a gpio and sets it high to set the SC55112
 * USER_OFF pin high.
 */
void gpio_pmic_poweroff(void)
{
	/*
	 * Configure GPIO C4 as low.
	 * This is connected to the SC55112 WDI input pin.
	 */
	local_irq_disable();

	/* Configure GPIO C4 as low */
	gpio_set_data(2, 4, 0);

	/* Wait for the power to go off.  Don't go anywhere.  */
	while (1) ;
}

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
	unsigned int pbc_bctrl1_set = 0;
	switch (module) {
	case 0:
		iomux_config_mux(SD1_CLK_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD1_CMD_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD1_DAT0_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD1_DAT1_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD1_DAT2_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD1_DAT3_PIN, MUX0_OUT, MUX0_IN);
		pbc_bctrl1_set |= PBC_BCTRL1_MCP1;
		break;
	case 1:
		iomux_config_mux(SD2_CLK_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD2_CMD_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD2_DAT0_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD2_DAT1_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD2_DAT2_PIN, MUX0_OUT, MUX0_IN);
		iomux_config_mux(SD2_DAT3_PIN, MUX0_OUT, MUX0_IN);
		pbc_bctrl1_set |= PBC_BCTRL1_MCP2;
		break;
	default:
		break;
	}
	__raw_writew(pbc_bctrl1_set, PBC_BASE_ADDRESS + PBC_BCTRL1_SET);
}

EXPORT_SYMBOL(gpio_sdhc_active);

/*!
 * Setup GPIO for SDHC1 to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module)
{
	unsigned int pbc_bctrl1_set = 0;
	switch (module) {
	case 0:
		iomux_config_mux(SD1_CLK_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD1_CMD_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD1_DAT0_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD1_DAT1_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD1_DAT2_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD1_DAT3_PIN, MUX0_OUT, NONE_IN);
		pbc_bctrl1_set &= PBC_BCTRL1_MCP1;
		break;
	case 1:
		iomux_config_mux(SD2_CLK_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD2_CMD_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD2_DAT0_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD2_DAT1_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD2_DAT2_PIN, MUX0_OUT, NONE_IN);
		iomux_config_mux(SD2_DAT3_PIN, MUX0_OUT, NONE_IN);
		pbc_bctrl1_set &= PBC_BCTRL1_MCP2;
		break;
	default:
		break;
	}
	__raw_writew(pbc_bctrl1_set, PBC_BASE_ADDRESS + PBC_BCTRL1_SET);
}

EXPORT_SYMBOL(gpio_sdhc_inactive);

/*
 * Probe for the card. If present the GPIO data would be set.
 */
int sdhc_find_card(void)
{
	return gpio_get_data(1, 16);
}

EXPORT_SYMBOL(sdhc_find_card);

/*!
 * Invert the IOMUX/GPIO for SDHC1 SD1_DET.
 *
 * @param flag Flag represents whether the card is inserted/removed.
 *             Using this sensitive level of GPIO signal is changed.
 *
 **/
void sdhc_intr_clear(int flag)
{
	if (flag) {
		set_irq_type(IOMUX_TO_IRQ(SD1_MMC_PU_CTRL_PIN), IRQT_FALLING);
	} else {
		set_irq_type(IOMUX_TO_IRQ(SD1_MMC_PU_CTRL_PIN), IRQT_RISING);
	}
}

EXPORT_SYMBOL(sdhc_intr_clear);

/*!
 * Setup the IOMUX/GPIO for SDHC1 SD1_DET.
 *
 * @param  host Pointer to MMC/SD host structure.
 * @param  handler      GPIO ISR function pointer for the GPIO signal.
 * @return The function returns 0 on success and -1 on failure.
 *
 **/
int sdhc_intr_setup(void *host,
		    irqreturn_t(*handler) (int, void *, struct pt_regs *))
{
	int ret;

	/* MMC1_SDDET is connected to GPIO37 */
	iomux_config_mux(SD1_MMC_PU_CTRL_PIN, MUX0_OUT, GPIO_MUX1_IN);

	/* check if a card in the slot if so we need to start with
	 * the proper edge definition
	 */
	sdhc_intr_clear(sdhc_find_card());

	ret = request_irq(IOMUX_TO_IRQ(SD1_MMC_PU_CTRL_PIN), handler,
			  0, "MXCMMC", host);
	return ret;
}

EXPORT_SYMBOL(sdhc_intr_setup);

/*!
 * Clear the IOMUX/GPIO for SDHC1 SD1_DET.
 */
void sdhc_intr_destroy(void *host)
{
	free_irq(IOMUX_TO_IRQ(SD1_MMC_PU_CTRL_PIN), host);
}

EXPORT_SYMBOL(sdhc_intr_destroy);

/*
 * Find the minimum clock for SDHC.
 *
 * @param clk SDHC module number.
 * @return Returns the minimum SDHC clock.
 */
unsigned int sdhc_get_min_clock(enum mxc_clocks clk)
{
	return (mxc_get_clocks(SDHC1_CLK) / 6) / 32;
}

EXPORT_SYMBOL(sdhc_get_min_clock);

/*
 * Find the maximum clock for SDHC.
 * @param clk SDHC module number.
 * @return Returns the maximum SDHC clock.
 */
unsigned int sdhc_get_max_clock(enum mxc_clocks clk)
{
	/* SDHC1 clock:8400000 */
	return mxc_get_clocks(SDHC1_CLK) / 2;
}

EXPORT_SYMBOL(sdhc_get_max_clock);

/*!
 * Setup GPIO for LCD to be active
 *
 */
void gpio_lcd_active(void)
{
	u16 pbc_bctrl1_set = 0;

	iomux_config_mux(IPU_LD0_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD1_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD2_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD3_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD4_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD5_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD6_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD7_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD8_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD9_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD10_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD11_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD12_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD13_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD14_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD15_PIN, MUX0_OUT, MUX0_IN);
	iomux_config_mux(IPU_LD17_D0_VSYNC_PIN, MUX0_OUT, MUX0_IN);	// LD17
	iomux_config_mux(IPU_D3_VSYNC_BE0_PIN, MUX0_OUT, MUX0_IN);	// VSYNC
	iomux_config_mux(IPU_D3_HSYNC_BE1_PIN, MUX0_OUT, MUX0_IN);	// HSYNC
	iomux_config_mux(IPU_D3_CLK_CONTR_PIN, MUX0_OUT, MUX0_IN);	// CLK
	iomux_config_mux(IPU_D3_DRDY_PS_PIN, MUX0_OUT, MUX0_IN);	// DRDY
	iomux_config_mux(IPU_REV_D1_CS_PIN, MUX0_OUT, MUX0_IN);	// REV
#ifdef CONFIG_FB_MXC_SHARP_QVGA_PANEL
	iomux_config_mux(IPU_LD16_D0_CS_PIN, MUX0_OUT, MUX0_IN);	// LD16
	iomux_config_mux(IPU_REV_D1_CS_PIN, MUX0_OUT, MUX0_IN);	// REV
	iomux_config_mux(IPU_D3_CONTR_PAR_RS_PIN, MUX0_OUT, MUX0_IN);	// CONTR
	iomux_config_mux(IPU_D3_SPL_RD_PIN, MUX0_OUT, MUX0_IN);	// SPL
	iomux_config_mux(IPU_D3_CLS_WR_PIN, MUX0_OUT, MUX0_IN);	// CLS
#else				// Epson or non-Sharp panel
	iomux_config_mux(IPU_LD16_D0_CS_PIN, MUX2_OUT, MUX2_IN);	// LD16
	iomux_config_mux(IPU_REV_D1_CS_PIN, MUX2_OUT, MUX2_IN);	// REV
	iomux_config_mux(IPU_D3_CONTR_PAR_RS_PIN, MUX2_OUT, MUX2_IN);	// CONTR
	iomux_config_mux(IPU_D3_SPL_RD_PIN, MUX2_OUT, MUX2_IN);	// SPL
	iomux_config_mux(IPU_D3_CLS_WR_PIN, MUX2_OUT, MUX2_IN);	// CLS
#endif
	pbc_bctrl1_set = (u16) PBC_BCTRL1_LCDON;
	__raw_writew(pbc_bctrl1_set, PBC_BASE_ADDRESS + PBC_BCTRL1_SET);
}

/*!
 * Setup GPIO for LCD to be inactive
 *
 */
void gpio_lcd_inactive(void)
{
	u16 pbc_bctrl1_set = 0;

	iomux_config_mux(IPU_LD0_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD1_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD2_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD3_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD4_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD5_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD6_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD7_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD8_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD9_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD10_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD11_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD12_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD13_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD14_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD15_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD16_D0_CS_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_LD17_D0_VSYNC_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_D3_VSYNC_BE0_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_D3_HSYNC_BE1_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_D3_CLK_CONTR_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_D3_DRDY_PS_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_REV_D1_CS_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_D3_CONTR_PAR_RS_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_D3_SPL_RD_PIN, GPIO_MUX1_OUT, NONE_IN);
	iomux_config_mux(IPU_D3_CLS_WR_PIN, GPIO_MUX1_OUT, NONE_IN);

	pbc_bctrl1_set = (u16) PBC_BCTRL1_LCDON;
	__raw_writew(pbc_bctrl1_set, PBC_BASE_ADDRESS + PBC_BCTRL1_SET + 2);
}

/*!
 * Setup GPIO for sensor to be active
 *
 */
void gpio_sensor_active(void)
{
	u16 temp;

	temp = PBC_BCTRL1_SENSORON;

	__raw_writew(temp, PBC_BASE_ADDRESS + PBC_BCTRL1_SET);

	iomux_config_mux(CSI_MCLK_PIN, MUX0_OUT, MUX0_IN);

}

#if 0
void slcd_gpio_config(void)
{
	edio_set_data(ED_INT4, 1);

	iomux_config_mux(ED_INT4_PIN, MUX0_OUT, MUX0_IN);
	edio_config(ED_INT4, true, EDIO_INT_NONE);

	edio_set_data(ED_INT4, 0);
	msleep(1);
	edio_set_data(ED_INT4, 1);
	msleep(1);
}
#endif
