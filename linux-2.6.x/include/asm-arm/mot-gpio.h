/*
 * mot-gpio.h - Motorola-hardware-specific GPIO API
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
 * 04-Oct-2006  Motorola        Initial revision.
 * 19-Oct-2006  Motorola        Added UI_IC_DBG and FSS_HYST for Elba.
 * 09-Nov-2006  Motorola        Added additional signals for ArgonLV.
 * 26-Nov-2006  Motorola        Added GPU_RESET for ArgonLV.
 * 06-Dec-2006  Motorola        Added etm_enable_trigger_clock export.
 * 11-Dec-2006  Motorola        UI_IC_DBG changes for Elba R1.2
 * 02-Jan-2007  Motorola        Added support for Lido P2.
 * 02-Jan-2007  Motorola        Update high speed USB API.
 * 07-Jan-2007  Motorola        Support P3C SCM-A11 wingboard.
 * 26-Jan-2007  Motorola        Bluetooth current drain improvements.
 * 12-Feb-2007  Motorola        Adapt to use hw config tree.
 * 13-Mar-2007  Motorola        Added more gpio_signal IRQ manipulators.
 * 20-Mar-2007  Motorola        Remove unused flip/slider functions.
 * 23-Mar-2007  Motorola        Add GPIO_SIGNAL_ANT_MMC_EN.
 * 19-Jul-2007  Motorola        Add dev_id to free_irq.
 */

#ifndef __ASM_ARM_MOT_GPIO_H
#define __ASM_ARM_MOT_GPIO_H

#include <linux/config.h>
#include <linux/compiler.h>

#if defined(CONFIG_MOT_FEAT_GPIO_API)

#if defined(CONFIG_MOT_FEAT_GPIO_API_SDHC)
#include <asm/arch/clock.h>
#endif

#include <asm/arch/gpio.h>

/* pick up the correct definition of iomux_config_mux for the SIERRA team */
#if defined(CONFIG_ARCH_MXC91131)
#include "../../arch/arm/mach-mxc91131/iomux.h"
#elif defined(CONFIG_ARCH_MXC91231)
#include "../../arch/arm/mach-mxc91231/iomux.h"
#elif defined(CONFIG_ARCH_MXC91321) || defined(CONFIG_ARCH_MXC91331)
#include "../../arch/arm/mach-mxc91321/iomux.h"
#endif


/**
 * Convert a port-signal pair into an MXC GPIO identifier. This is based
 * on code from arch/arm/plat-mxc/gpio.c.
 */
#define GPIO_PORT_SIG_TO_MXC_GPIO(p, s) ((p*32)+s)


/* ************************************************************************
 * Generic GPIO port, value, and direction definitions.
 * ***********************************************************************/
#define GPIO_INVALID_PORT   0xFFFFFFFF

#define GPIO_DATA_LOW       0
#define GPIO_DATA_HIGH      1
#define GPIO_DATA_INVALID   0xFFFF

/** 
 * Unlike GPIO_DATA_(HIGH|LOW), these are indifferent to polarity.
 * Use these to specify whether a logical RESET should be asserted. 
 * The reset could be a physical high or low of the GPIO pin, 
 * but the corresponding GPIO function should take care of mapping 
 * ASSERT_GPIO_SIGNAL to either GPIO_DATA_LOW or GPIO_DATA_HIGH.
 */
enum gpio_signal_assertion {
    DEASSERT_GPIO_SIGNAL    = 0,
    ASSERT_GPIO_SIGNAL
};

#define GPIO_GDIR_INPUT     0
#define GPIO_GDIR_OUTPUT    1
#define GPIO_GDIR_INVALID   0xFFFF


/* ************************************************************************
 * IOMUX definitions
 * ***********************************************************************/
#define IOMUX_INVALID_PIN   0xFFFFFFFF


/* ************************************************************************
 * Description of GPIO signal and initial settings.
 * ***********************************************************************/

/**
 * This enumeration is used as an index into the gpio_signal_mapping array
 * as define in mot-gpio.c.  The values of the enumeration do not map
 * directly to the values in the hardware configuration tree, see mot-gpio.c
 * for more information.
 * 
 * MAX_GPIO_SIGNAL should always be the last name defined in the enum.
 */
enum gpio_signal {
    GPIO_SIGNAL_APP_CLK_EN_B = 0,
    GPIO_SIGNAL_UART_DETECT,
    GPIO_SIGNAL_U1_TXD,
    GPIO_SIGNAL_U1_CTS_B,
    GPIO_SIGNAL_VFUSE_EN,
    GPIO_SIGNAL_LIN_VIB_AMP_EN,
    GPIO_SIGNAL_VVIB_EN,
    GPIO_SIGNAL_WDOG_AP,
    GPIO_SIGNAL_WDOG_B,
    GPIO_SIGNAL_LOBAT_B,
    GPIO_SIGNAL_POWER_DVS0,
    GPIO_SIGNAL_POWER_RDY,
    GPIO_SIGNAL_BT_HOST_WAKE_B,
    GPIO_SIGNAL_BT_POWER,
    GPIO_SIGNAL_BT_RESET_B,
    GPIO_SIGNAL_BT_WAKE_B,
    GPIO_SIGNAL_CAM_EXT_PWRDN,
    GPIO_SIGNAL_CAM_FLASH_EN,
    GPIO_SIGNAL_CAM_INT_PWRDN,
    GPIO_SIGNAL_CAM_PD,
    GPIO_SIGNAL_CAM_RST_B,
    GPIO_SIGNAL_CAM_TORCH_EN,
    GPIO_SIGNAL_CLI_BKL,
    GPIO_SIGNAL_CLI_RST_B,
    GPIO_SIGNAL_DISP_RST_B,
    GPIO_SIGNAL_DISP_SD,
    GPIO_SIGNAL_LCD_BACKLIGHT,
    GPIO_SIGNAL_LCD_SD,
    GPIO_SIGNAL_DISP_CM,
    GPIO_SIGNAL_SERDES_RESET_B,
    GPIO_SIGNAL_STBY,
    GPIO_SIGNAL_SER_EN,
    GPIO_SIGNAL_EL_EN,
    GPIO_SIGNAL_CAP_RESET,
    GPIO_SIGNAL_FLIP_OPEN,
    GPIO_SIGNAL_FLIP_DETECT,
    GPIO_SIGNAL_SLIDER_OPEN,
    GPIO_SIGNAL_KEYS_LOCKED,
    GPIO_SIGNAL_TOUCH_INTB,
    GPIO_SIGNAL_FM_INT,
    GPIO_SIGNAL_FM_RST_B,
    GPIO_SIGNAL_FM_INTERRUPT,
    GPIO_SIGNAL_FM_RESET,
    GPIO_SIGNAL_GPS_PWR_EN,
    GPIO_SIGNAL_GPS_RESET,
    GPIO_SIGNAL_GPU_A2,
    GPIO_SIGNAL_GPU_CORE1_EN,
    GPIO_SIGNAL_GPU_DPD,
    GPIO_SIGNAL_GPU_INT_B,
    GPIO_SIGNAL_GPU_RESET,
    GPIO_SIGNAL_MEGA_SIM_EN,
    GPIO_SIGNAL_SD1_DAT3,
    GPIO_SIGNAL_SD1_DET_B,
    GPIO_SIGNAL_SD2_DAT3,
    GPIO_SIGNAL_TF_DET,
    GPIO_SIGNAL_TNLC_KCHG,
    GPIO_SIGNAL_TNLC_KCHG_INT,
    GPIO_SIGNAL_TNLC_RCHG,
    GPIO_SIGNAL_TNLC_RESET,
    GPIO_SIGNAL_FSS_HYST,
    GPIO_SIGNAL_UI_IC_DBG,
    GPIO_SIGNAL_USB_HS_DMA_REQ,
    GPIO_SIGNAL_USB_HS_FLAGC,
    GPIO_SIGNAL_USB_HS_INT,
    GPIO_SIGNAL_USB_HS_RESET,
    GPIO_SIGNAL_USB_HS_SWITCH,
    GPIO_SIGNAL_USB_HS_WAKEUP,
    GPIO_SIGNAL_USB_XCVR_EN,
    GPIO_SIGNAL_WLAN_CLIENT_WAKE_B,
    GPIO_SIGNAL_WLAN_DBHD_MUX,
    GPIO_SIGNAL_WLAN_HOST_WAKE_B,
    GPIO_SIGNAL_WLAN_PWR_DWN_B,
    GPIO_SIGNAL_WLAN_RESET,
    GPIO_SIGNAL_PWM_BKL,
    GPIO_SIGNAL_MAIN_BKL,
    GPIO_SIGNAL_IRDA_SD,
    GPIO_SIGNAL_ANT_MMC_EN,
    MAX_GPIO_SIGNAL
};


/**
 * Structure describing the configuration of a GPIO signal.
 */
struct gpio_signal_description {
    __u32   port;
    __u32   sig_no;
};


/**
 * Description of GPIO signal and initial settings. Must be defined for
 * each variety of board.
 */
extern struct gpio_signal_description gpio_signal_mapping[MAX_GPIO_SIGNAL];


/**
 * Determine if index is a valid index into the gpio_signal_mapping
 * array and if the GPIO signal associated with the index is valid for
 * that hardware configuration.
 */
#define GPIO_SIGNAL_IS_VALID(index) \
    ( (index >= 0) && (index < MAX_GPIO_SIGNAL) \
      && (gpio_signal_mapping[index].port != GPIO_INVALID_PORT) )


/* ************************************************************************
 * Functions used during hardware initialization.
 * ***********************************************************************/
extern enum iomux_pins hwcfg_pin_to_fsl_pin(u16 schemavalue) __init;
extern int mot_iomux_mux_init(void) __init;
extern int mot_iomux_pad_init(void) __init;
extern void arch_mot_iomux_pad_init(u16 grp, u16 config) __init;

extern int mot_gpio_init(void) __init;
extern int mot_gpio_update_mappings(void) __init;


/* ************************************************************************
 * Functions to manipulate GPIO settings based on the
 * gpio_signal_mapping array.
 * ***********************************************************************/
extern int gpio_signal_config(enum gpio_signal index, bool out,
        enum gpio_int_cfg icr);

extern int gpio_signal_request_irq(enum gpio_signal index,
        enum gpio_prio prio, gpio_irq_handler handler, __u32 irq_flags,
        const char *devname, void *dev_id);
extern int gpio_signal_free_irq(enum gpio_signal index,
        enum gpio_prio prio, void *dev_id);
extern int gpio_signal_disable_irq(enum gpio_signal index);
extern int gpio_signal_enable_irq(enum gpio_signal index);

extern int gpio_signal_clear_int(enum gpio_signal index);

extern int gpio_signal_set_data(enum gpio_signal index, __u32 data);
extern int gpio_signal_get_data(enum gpio_signal index, __u32 *data);

extern __u32 gpio_signal_get_data_check(enum gpio_signal index);


/* ************************************************************************
 * Common GPIO API.
 * ***********************************************************************/
extern void gpio_uart_active(int port, int no_irda);
extern void gpio_uart_inactive(int port, int no_irda);
extern void config_uartdma_event(int port);
extern void gpio_keypad_active(void);
extern void gpio_keypad_inactive(void);
extern void gpio_spi_active(int cspi_mod);
extern void gpio_spi_inactive(int cspi_mod);
extern void gpio_i2c_active(int i2c_num);
extern void gpio_i2c_inactive(int i2c_num);


#ifdef CONFIG_MOT_FEAT_GPIO_TRACE
/* ************************************************************************
 * GPIO trace messaging -- for debug messages
 * ***********************************************************************/

#define gpio_tracemsg(fmt, args...)  printk(KERN_ALERT fmt "\n", ##args)

#else

#define gpio_tracemsg(fmt, args...)

#endif /* CONFIG_MOT_FEAT_GPIO_TRACE */


#if defined(CONFIG_MOT_FEAT_GPIO_API_MC13783)
/* ************************************************************************
 * Atlas-related GPIO functions.
 * ***********************************************************************/

extern void gpio_mc13783_active(void);
extern void gpio_mc13783_clear_int(void);
extern int  gpio_mc13783_get_spi(void); 
extern int  gpio_mc13783_get_ss(void);

#endif /* CONFIG_MOT_FEAT_GPIO_API_MC13783 */


#if defined(CONFIG_MOT_FEAT_GPIO_API_BTPOWER)
/* ************************************************************************
 * Bluetooth power management GPIO API
 * ***********************************************************************/

/* BT_HOST_WAKE_B signal */
extern int   gpio_bluetooth_hostwake_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id);
extern void  gpio_bluetooth_hostwake_free_irq(void *dev_id);
extern void  gpio_bluetooth_hostwake_clear_int(void);
extern __u32 gpio_bluetooth_hostwake_get_data(void);

/* BT_WAKE_B signal */
extern void  gpio_bluetooth_wake_set_data(__u32 data);
extern __u32 gpio_bluetooth_wake_get_data(void);

/* BT_POWER signal */
extern void  gpio_bluetooth_power_set_data(__u32 data);
extern __u32 gpio_bluetooth_power_get_data(void);

#endif /* CONFIG_MOT_FEAT_GPIO_API_BTPOWER */


#if defined(CONFIG_MOT_FEAT_GPIO_API_CSI)
/* ************************************************************************
 * Camera Sensor Interface GPIO control
 * ***********************************************************************/

extern void gpio_sensor_active(void);
extern void gpio_sensor_inactive(void);

#endif /* CONFIG_MOT_FEAT_GPIO_API_CSI */


#if defined(CONFIG_MOT_FEAT_GPIO_API_DAI)
/* ************************************************************************
 * Digital Audio Interface
 * ***********************************************************************/

extern void gpio_dai_enable(void);
extern void gpio_dai_disable(void);

#endif /* CONFIG_MOT_FEAT_GPIO_API_DAI */


#if defined(CONFIG_MOT_FEAT_GPIO_API_ETHERNET)
/* ************************************************************************
 * Ethernet Interrupt
 * ***********************************************************************/
extern int  enet_request_irq(
        irqreturn_t (*handler)(int, void *, struct pt_regs *),
        unsigned long irq_flags, const char * devname, void *dev_id);
extern int  enet_free_irq(void *dev_id);
extern void enet_clear_int(void);

#endif /* CONFIG_MOT_FEAT_GPIO_API_ETHERNET */


#if defined(CONFIG_MOT_FEAT_GPIO_API_ETM)
/* ************************************************************************
 * ETM IOMUX Setting Control
 * ***********************************************************************/

enum etm_iomux {
    ETM_MUX_DEFAULT,
    ETM_MUX_IPU,
    ETM_MUX_CSI_KPP
};

extern void etm_iomux_config(enum etm_iomux alternative);
extern void etm_enable_trigger_clock(void);

#endif /* CONFIG_MOT_FEAT_GPIO_API_ETM */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LCD)
/* ************************************************************************
 * LCD control
 * ***********************************************************************/
extern void gpio_lcd_active(void);
extern void gpio_lcd_inactive(void);
#endif /* CONFIG_MOT_FEAT_GPIO_API_LCD */


#if defined(CONFIG_MOT_FEAT_GPIO_API_SERIALIZER)
/* ************************************************************************
 * LCD serializer control
 * ***********************************************************************/
extern void gpio_lcd_serializer_reset(enum gpio_signal_assertion asserted);
extern void gpio_lcd_serializer_stby(enum gpio_signal_assertion asserted);
#endif /* CONFIG_MOT_FEAT_GPIO_API_SERIALIZER */


#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING)
/* ************************************************************************
 * Lighting control
 * ***********************************************************************/

#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_CAM_TORCH)
/* Camera Torch */
extern void gpio_camera_torch_enable(int enable);
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_CAM_TORCH */

#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_CAM_FLASH)
/* Camera Flash */
extern void gpio_camera_flash_enable(int enable);
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_CAM_FLASH */

#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_EL)
/* Keypad Backlights */
extern void gpio_backlight_numbers_enable(int enable);
extern void gpio_backlight_navigation_enable(int enable);
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_EL */

#if defined(CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD)
/* Display Backlight */
extern void gpio_lcd_backlight_enable(bool enable);
extern int  gpio_get_lcd_backlight(void);

#if defined(CONFIG_ARCH_MXC91321) /* ArgonLV */
extern void pwm_set_lcd_bkl_brightness(int value);
extern int  pwm_get_lcd_bkl_brightness(void);
#endif /* CONFIG_ARCH_MXC91321 */
#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING_LCD */

#endif /* CONFIG_MOT_FEAT_GPIO_API_LIGHTING */

#if defined(CONFIG_MOT_FEAT_GPIO_API_SDHC)
/* ************************************************************************
 * Secure-Digital Host Controller API
 * ***********************************************************************/
extern void         gpio_sdhc_active(int module);
extern void         gpio_sdhc_inactive(int module);
extern int          sdhc_intr_setup(int module, void *host,
        gpio_irq_handler handler);
extern void         sdhc_intr_destroy(int module, void *host);
extern void         sdhc_intr_clear(int module, int flag);
extern unsigned int sdhc_get_min_clock(enum mxc_clocks clk);
extern unsigned int sdhc_get_max_clock(enum mxc_clocks clk);
extern int          sdhc_find_card(int module);

#endif /* CONFIG_MOT_FEAT_GPIO_API_SDHC */


#if defined(CONFIG_MOT_FEAT_GPIO_API_USBHS)
/* ************************************************************************
 * Highspeed USB API
 * ***********************************************************************/

/* USB_HS_RESET */
extern void  gpio_usb_hs_reset_set_data(__u32 enable);

/* USB_HS_WAKEUP */
extern void  gpio_usb_hs_wakeup_set_data(__u32 wakeup);

/* USB_HS_SWITCH */
extern void  gpio_usb_hs_switch_set_data(__u32 swtch);

/* USB_HS_FLAGC */
extern int   gpio_usb_hs_flagc_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id);
extern void  gpio_usb_hs_flagc_free_irq(void *dev_id);
extern void  gpio_usb_hs_flagc_clear_int(void);
extern __u32 gpio_usb_hs_flagc_get_data(void);

/* USB_HS_INT */
extern int   gpio_usb_hs_int_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id);
extern void  gpio_usb_hs_int_free_irq(void *dev_id);
extern void  gpio_usb_hs_int_clear_int(void);
extern __u32 gpio_usb_hs_int_get_data(void);

/* USB_HS_DMA_REQ */
extern void  gpio_usb_hs_dma_req_config_gpio_mode(void);
extern void  gpio_usb_hs_dma_req_config_sdma_mode(void);
extern void  gpio_usb_hs_dma_req_config_dual_mode(void);
extern int   gpio_usb_hs_dma_req_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id);
extern void  gpio_usb_hs_dma_req_set_irq_type(gpio_edge_t edge);
extern void  gpio_usb_hs_dma_req_free_irq(void *dev_id);
extern void  gpio_usb_hs_dma_req_clear_int(void);
extern __u32 gpio_usb_hs_dma_req_get_data(void);

#endif /* CONFIG_MOT_FEAT_GPIO_API_USBHS */


#if defined(CONFIG_MOT_FEAT_GPIO_API_WLAN)
/* ************************************************************************
 * Control WLAN GPIO signals.
 * ***********************************************************************/

/* WLAN_RESET */
extern void  gpio_wlan_reset_set_data(__u32 data);

/* WLAN_CLIENT_WAKE_B */
extern void  gpio_wlan_clientwake_set_data(__u32 data);

/* WLAN_PWR_DWN_B */
extern __u32 gpio_wlan_powerdown_get_data(void);
extern void  gpio_wlan_powerdown_set_data(__u32 data);

/* WLAN_HOST_WAKE_B */
extern int  gpio_wlan_hostwake_request_irq(gpio_irq_handler handler,
        unsigned long irq_flags, const char *devname, void *dev_id);
extern void  gpio_wlan_hostwake_free_irq(void *dev_id);
extern void  gpio_wlan_hostwake_clear_int(void);
extern __u32 gpio_wlan_hostwake_get_data(void);

#endif /* CONFIG_MOT_FEAT_GPIO_API_WLAN */


#if defined(CONFIG_MOT_FEAT_GPIO_API_EDIO)
/* ************************************************************************
 * Deprecated Freescale EDIO API
 * ***********************************************************************/

/**
 * This enumeration data type defines various signals for interrupting the
 * ARM core from EDIO signals. 
 */
enum edio_sig_no {
        ED_INT0 = 0,
        ED_INT1 = 1,
        ED_INT2 = 2,
        ED_INT3 = 3,
        ED_INT4 = 4,
        ED_INT5 = 5,
        ED_INT6 = 6,
        ED_INT7 = 7,
};


/**
 * Highest EDIO signal.
 */
#define EDIO_MAX_SIG_NO     ED_INT7


/**
 * EDIO data register low
 */
#define EDIO_DATA_LOW   0


/**
 * EDIO data register high
 */
#define EDIO_DATA_HIGH  1


extern void __deprecated    edio_set_data(enum edio_sig_no sig_no, __u32 data);
extern __u32                edio_get_data(enum edio_sig_no sig_no);

#endif /* CONFIG_MOT_FEAT_GPIO_API_EDIO */

#endif /* CONFIG_MOT_FEAT_GPIO_API */

#endif  /* __ASM_ARM_MOT_GPIO_H */
