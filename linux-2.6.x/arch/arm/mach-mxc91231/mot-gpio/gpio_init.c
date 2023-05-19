/*
 * linux/arch/arm/mach-mxc91231/mot-gpio/gpio_init.c
 *
 * Initial GPIO register settings for SCM-A11 platform.
 *
 * Copyright 2006-2007 Motorola, Inc.
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
 * 19-Oct-2006  Motorola        Initial revision.
 * 02-Jan-2007  Motorola        Added support for Lido P2.
 * 31-Jan-2007  Motorola        Bluetooth current drain improvements.
 * 20-Feb-2007  Motorola        Update to use HWCFG tree.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/mot-gpio.h>

#include "mot-gpio-scma11.h"

/**
 * This function is called by mxc_board_init() at boot time.
 */
void __init scma11phone_gpio_init(void)
{
    /* set iomux pad registers to the prescribed state */
    mot_iomux_pad_init();

    /* read the GPIO signal mappings from the HWCFG tree */
    mot_gpio_update_mappings();

    /* configure GPIO registers to desired initial state */
    mot_gpio_init();
    
    /* configure IOMUX settings to their prescribed initial state */
    mot_iomux_mux_init();

#if defined(CONFIG_MOT_FEAT_GPIO_API_BTPOWER)
    /* disable UART1 for Bluetooth current drain improvement */
    gpio_bluetooth_power_set_data(0);
#endif

    return;
}
