/*
 * linux/arch/arm/mach-mxc91231/mot-gpio/iomux_init.c
 *
 * Initial IOMUX register settings for SCM-A11 platform.
 *
 * Copyright 2007 Motorola, Inc.
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
 * 30-Jan-2007  Motorola        Initial revision.
 * 20-Feb-2007  Motorola        Updated for HWCFG tree.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/mot-gpio.h>
#include <asm/mothwcfg.h>

#include "mot-gpio-scma11.h"

/**
 * The bootloader initializes the SDRAM IOMUX pad registers.
 */
#define IOMUX_PAD_SETTING_START 9


/**
 * The number of entries in the hwcfg_pins array.
 */
#define HWCFG_IOMUX_PIN_COUNT   145


/**
 * The first SCM-A11 pin is mapped to 0x01B6 in the HWCFG schema.
 */
#define HWCFG_SCMA11_PIN_BASE   0x01B6


/**
 * The first SCM-A11 pad group is mapped to 0x0200 in the HWCFG schema.
 */
#define HWCFG_SCMA11_PAD_BASE   0x0200


/**
 * Used to map hardware configuration tree IOMUX pin identifiers to the
 * identifier use by Freescale.
 */
static enum iomux_pins __initdata hwcfg_pins[HWCFG_IOMUX_PIN_COUNT] = {
    AP_CLE,                      /* 0x01B6 */
    AP_ALE,                      /* 0x01B7 */
    AP_CE_B,                     /* 0x01B8 */
    AP_RE_B,                     /* 0x01B9 */
    AP_WE_B,                     /* 0x01BA */
    AP_WP_B,                     /* 0x01BB */
    AP_BSY_B,                    /* 0x01BC */
    AP_U1_TXD,                   /* 0x01BD */
    AP_U1_RXD,                   /* 0x01BE */
    AP_U1_RTS_B,                 /* 0x01BF */
    AP_U1_CTS_B,                 /* 0x01C0 */
    AP_AD1_TXD,                  /* 0x01C1 */
    AP_AD1_RXD,                  /* 0x01C2 */
    AP_AD1_TXC,                  /* 0x01C3 */
    AP_AD1_TXFS,                 /* 0x01C4 */
    AP_AD2_TXD,                  /* 0x01C5 */
    AP_AD2_RXD,                  /* 0x01C6 */
    AP_AD2_TXC,                  /* 0x01C7 */
    AP_AD2_TXFS,                 /* 0x01C8 */
    AP_OWDAT,                    /* 0x01C9 */
    AP_IPU_LD17,                 /* 0x01CA */
    AP_IPU_D3_VSYNC,             /* 0x01CB */
    AP_IPU_D3_HSYNC,             /* 0x01CC */
    AP_IPU_D3_CLK,               /* 0x01CD */
    AP_IPU_D3_DRDY,              /* 0x01CE */
    AP_IPU_D3_CONTR,             /* 0x01CF */
    AP_IPU_D0_CS,                /* 0x01D0 */
    AP_IPU_LD16,                 /* 0x01D1 */
    AP_IPU_D2_CS,                /* 0x01D2 */
    AP_IPU_PAR_RS,               /* 0x01D3 */
    AP_IPU_D3_PS,                /* 0x01D4 */
    AP_IPU_D3_CLS,               /* 0x01D5 */
    AP_IPU_RD,                   /* 0x01D6 */
    AP_IPU_WR,                   /* 0x01D7 */
    AP_IPU_LD0,                  /* 0x01D8 */
    AP_IPU_LD1,                  /* 0x01D9 */
    AP_IPU_LD2,                  /* 0x01DA */
    AP_IPU_LD3,                  /* 0x01DB */
    AP_IPU_LD4,                  /* 0x01DC */
    AP_IPU_LD5,                  /* 0x01DD */
    AP_IPU_LD6,                  /* 0x01DE */
    AP_IPU_LD7,                  /* 0x01DF */
    AP_IPU_LD8,                  /* 0x01E0 */
    AP_IPU_LD9,                  /* 0x01E1 */
    AP_IPU_LD10,                 /* 0x01E2 */
    AP_IPU_LD11,                 /* 0x01E3 */
    AP_IPU_LD12,                 /* 0x01E4 */
    AP_IPU_LD13,                 /* 0x01E5 */
    AP_IPU_LD14,                 /* 0x01E6 */
    AP_IPU_LD15,                 /* 0x01E7 */
    AP_KPROW4,                   /* 0x01E8 */
    AP_KPROW5,                   /* 0x01E9 */
    AP_GPIO_AP_B17,              /* 0x01EA */
    AP_GPIO_AP_B18,              /* 0x01EB */
    AP_KPCOL3,                   /* 0x01EC */
    AP_KPCOL4,                   /* 0x01ED */
    AP_KPCOL5,                   /* 0x01EE */
    AP_GPIO_AP_B22,              /* 0x01EF */
    AP_GPIO_AP_B23,              /* 0x01F0 */
    AP_CSI_D0,                   /* 0x01F1 */
    AP_CSI_D1,                   /* 0x01F2 */
    AP_CSI_D2,                   /* 0x01F3 */
    AP_CSI_D3,                   /* 0x01F4 */
    AP_CSI_D4,                   /* 0x01F5 */
    AP_CSI_D5,                   /* 0x01F6 */
    AP_CSI_D6,                   /* 0x01F7 */
    AP_CSI_D7,                   /* 0x01F8 */
    AP_CSI_D8,                   /* 0x01F9 */
    AP_CSI_D9,                   /* 0x01FA */
    AP_CSI_MCLK,                 /* 0x01FB */
    AP_CSI_VSYNC,                /* 0x01FC */
    AP_CSI_HSYNC,                /* 0x01FD */
    AP_CSI_PIXCLK,               /* 0x01FE */
    AP_I2CLK,                    /* 0x01FF */
    AP_I2DAT,                    /* 0x0200 */
    AP_GPIO_AP_C8,               /* 0x0201 */
    AP_GPIO_AP_C9,               /* 0x0202 */
    AP_GPIO_AP_C10,              /* 0x0203 */
    AP_GPIO_AP_C11,              /* 0x0204 */
    AP_GPIO_AP_C12,              /* 0x0205 */
    AP_GPIO_AP_C13,              /* 0x0206 */
    AP_GPIO_AP_C14,              /* 0x0207 */
    AP_GPIO_AP_C15,              /* 0x0208 */
    AP_GPIO_AP_C16,              /* 0x0209 */
    AP_GPIO_AP_C17,              /* 0x020A */
    AP_ED_INT0,                  /* 0x020B */
    AP_ED_INT1,                  /* 0x020C */
    AP_ED_INT2,                  /* 0x020D */
    AP_ED_INT3,                  /* 0x020E */
    AP_ED_INT4,                  /* 0x020F */
    AP_ED_INT5,                  /* 0x0210 */
    AP_ED_INT6,                  /* 0x0211 */
    AP_ED_INT7,                  /* 0x0212 */
    AP_U2_DSR_B,                 /* 0x0213 */
    AP_U2_RI_B,                  /* 0x0214 */
    AP_U2_CTS_B,                 /* 0x0215 */
    AP_U2_DTR_B,                 /* 0x0216 */
    AP_KPROW0,                   /* 0x0217 */
    AP_KPROW1,                   /* 0x0218 */
    AP_KPROW2,                   /* 0x0219 */
    AP_KPROW3,                   /* 0x021A */
    AP_KPCOL0,                   /* 0x021B */
    AP_KPCOL1,                   /* 0x021C */
    AP_KPCOL2,                   /* 0x021D */
    SP_U3_TXD,                   /* 0x021E */
    SP_U3_RXD,                   /* 0x021F */
    SP_U3_RTS_B,                 /* 0x0220 */
    SP_U3_CTS_B,                 /* 0x0221 */
    SP_USB_TXOE_B,               /* 0x0222 */
    SP_USB_DAT_VP,               /* 0x0223 */
    SP_USB_SE0_VM,               /* 0x0224 */
    SP_USB_RXD,                  /* 0x0225 */
    SP_UH2_TXOE_B,               /* 0x0226 */
    SP_UH2_SPEED,                /* 0x0227 */
    SP_UH2_SUSPEND,              /* 0x0228 */
    SP_UH2_TXDP,                 /* 0x0229 */
    SP_UH2_RXDP,                 /* 0x022A */
    SP_UH2_RXDM,                 /* 0x022B */
    SP_UH2_OVR,                  /* 0x022C */
    SP_UH2_PWR,                  /* 0x022D */
    SP_SD1_DAT0,                 /* 0x022E */
    SP_SD1_DAT1,                 /* 0x022F */
    SP_SD1_DAT2,                 /* 0x0230 */
    SP_SD1_DAT3,                 /* 0x0231 */
    SP_SD1_CMD,                  /* 0x0232 */
    SP_SD1_CLK,                  /* 0x0233 */
    SP_SD2_DAT0,                 /* 0x0234 */
    SP_SD2_DAT1,                 /* 0x0235 */
    SP_SD2_DAT2,                 /* 0x0236 */
    SP_SD2_DAT3,                 /* 0x0237 */
    SP_GPIO_Shared26,            /* 0x0238 */
    SP_SPI1_CLK,                 /* 0x0239 */
    SP_SPI1_MOSI,                /* 0x023A */
    SP_SPI1_MISO,                /* 0x023B */
    SP_SPI1_SS0,                 /* 0x023C */
    SP_SPI1_SS1,                 /* 0x023D */
    SP_SD2_CMD,                  /* 0x023E */
    SP_SD2_CLK,                  /* 0x023F */
    SP_SIM1_RST_B,               /* 0x0240 */
    SP_SIM1_SVEN,                /* 0x0241 */
    SP_SIM1_CLK,                 /* 0x0242 */
    SP_SIM1_TRXD,                /* 0x0243 */
    SP_SIM1_PD,                  /* 0x0244 */
    SP_UH2_TXDM,                 /* 0x0245 */
    SP_UH2_RXD,                  /* 0x0246 */
};


/**
 * Convert the value defined in the hardware config schema for an IOMUX
 * pin to the Freescale defined value for the same pin.
 *
 * @param   schemavalue Value assocated with a pin name in hwcfg.
 * 
 * @return  Freescale pin name for the pin.
 */
enum iomux_pins __init hwcfg_pin_to_fsl_pin(u16 schemavalue)
{
    /* SCM-A11 pins begin at 0x01B6 in the device tree schema */
    schemavalue -= HWCFG_SCMA11_PIN_BASE;

    if( schemavalue < HWCFG_IOMUX_PIN_COUNT ) {
        return hwcfg_pins[schemavalue];
    } else {
        return IOMUX_INVALID_PIN;
    }
}


/**
 * Set an IOMUX pad group register based on data passed in from HWCFG.
 *
 * @param   grp     HWCFG identifier for pad group.
 * @param   config  HWCFG identifier for pad setting.
 */
void __init arch_mot_iomux_pad_init(u16 grp, u16 config)
{
    /* SCM-A11 HWCFG pad group identifiers begin at 0x0200 */
    grp -= HWCFG_SCMA11_PAD_BASE;

    /* the bootloader initializes the SDRAM pad settings */
    if(grp >= IOMUX_PAD_SETTING_START) {
        gpio_tracemsg("Setting pad register 0x%04x to: 0x%04x", grp, config);

        iomux_set_pad(grp, config);
    }
}
