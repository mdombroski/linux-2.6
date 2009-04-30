/*
 * linux/arch/arm/mach-at91/board-aixle.c
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2007 Atmel Corporation.
 *  Copyright (C) 2009 4D Electronics
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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/nrf24l01.h>
#include <linux/workqueue.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/gpio_backlight.h>

#include <video/atmel_lcdc.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/spi-gpio.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_shdwc.h>

#include "sam9_smc.h"
#include "generic.h"


static void __init aixle_map_io(void)
{
	/* Initialize processor: 12.00 MHz crystal */
	at91sam9263_initialize(12000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}


static void __init aixle_init_irq(void)
{
	at91sam9263_init_interrupts(NULL);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata aixle_usbh_data = {
	.ports		= 2,
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata aixle_udc_data = {
	.vbus_pin	= AT91_PIN_PB11,

	/* pull-up driven by UDC */
	.pullup_pin	= 0,
};


/*
 * ADS7846 Touchscreen
 */
#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
static int ads7846_pendown_state(void)
{
	/* Touchscreen PENIRQ */
	return !at91_get_gpio_value(AT91_PIN_PA14);
}

static struct ads7846_platform_data ads_info = {
	.model			= 7846,
	.x_min			= 150,
	.x_max			= 3830,
	.y_min			= 190,
	.y_max			= 3830,
	.vref_delay_usecs	= 100,
	.x_plate_ohms		= 550,
	.y_plate_ohms		= 300,
	.pressure_max		= 15000,
	.debounce_max		= 1,
	.debounce_rep		= 0,
	.debounce_tol		= (~0),
	.get_pendown_state	= ads7846_pendown_state,
};

static void __init aixle_add_device_ts(void)
{
	/* External IRQ1, with pullup */
	at91_set_B_periph(AT91_PIN_PA14, 1);
	at91_set_deglitch(AT91_PIN_PA14, 1);

	/* Touchscreen BUSY signal */
	at91_set_gpio_input(AT91_PIN_PB17, 1);
	at91_set_deglitch(AT91_PIN_PB17, 1);

}
#else
static void __init aixle_add_device_ts(void) {}
#endif


/*
 * Nordic nRF24L01 radio
 */
#if defined(CONFIG_NORDIC_NRF24L01) || defined(CONFIG_NORDIC_NRF24L01_MODULE)
static void nrf_set_rf_enable ( int state )
{
	// Set enable signal depending on state
	at91_set_gpio_output(AT91_PIN_PA13, state);
}

static struct nrf24l01_platform_data nrf_info = {
	.set_rf_enable	= nrf_set_rf_enable,
};

static void __init aixle_add_device_nrf(void)
{
	// configure irq pin (peripheral B, deglitch)
	at91_set_B_periph(AT91_PIN_PA15, 1);
	at91_set_deglitch(AT91_PIN_PA15, 1);

	// configure rf_enable pin (output, low)
	at91_set_gpio_output(AT91_PIN_PA13, 0);
}
#else
static void __init aixle_add_device_nrf(void) {}
#endif


/*
 * SPI devices
 *
 * bus_num=0 - Hardware spi, port 0
 * bus_num=1 - Hardware spi, port 1
 * bus_num=2 - GPIO spi
 */
static struct spi_board_info aixle_spi_devices[] = {
#if !defined(CONFIG_MMC_AT91)
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 20 * 1000 * 1000,
		.bus_num	= 0,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	{
		/* Sample Rate of ADS7846/TSC2046
		 * http://focus.ti.com/lit/an/sbaa155a/sbaa155a.pdf
		 */
		.modalias	= "ads7846",
		.chip_select	= 1,
		.max_speed_hz	= 2000 * 16,  /* (max sample rate @ 3V) * (cmd + data + overhead) */
//		.max_speed_hz	= 10000, /* really slow, but lots more reliable. */
		.bus_num	= 2,
		.platform_data	= &ads_info,
		.irq		= AT91SAM9263_ID_IRQ0,
	},
#endif
#if defined(CONFIG_NORDIC_NRF24L01) || defined(CONFIG_NORDIC_NRF24L01_MODULE)
	{
		.modalias	= "nrf24l01",
		.chip_select	= 0,
		.max_speed_hz	= 5 * 1000 * 1000,
		.bus_num	= 2,
		.platform_data	= &nrf_info,
		.irq		= AT91SAM9263_ID_IRQ1,
	},
#endif
};


/*
 * Bit-bang SPI configuration
 */
#if defined(CONFIG_SPI_AT91_GPIO) || defined(CONFIG_SPI_AT91_GPIO_MODULE)
static void aixle_spi_gpio_cs(struct at91_spigpio_info *spi, int cs, int value)
{
	int pin;
	switch (cs) {
		case 3:
			pin = AT91_PIN_PB18;
			break;
		case 2:
			pin = AT91_PIN_PB17;
			break;
		case 1:
			pin = AT91_PIN_PB16;
			break;
		case 0:
			pin = AT91_PIN_PB15;
			break;
		default:
			return;
	}

	switch (value) {
		case BITBANG_CS_ACTIVE:
			at91_set_gpio_output(pin, 0);
			break;
		case BITBANG_CS_INACTIVE:
			at91_set_gpio_output(pin, 1);
			break;
	}
}


static struct at91_spigpio_info spi_gpio_cfg = {
	.pin_clk	= AT91_PIN_PB14,
	.pin_mosi	= AT91_PIN_PB13,
	.pin_miso	= AT91_PIN_PB12,
	.chip_select	= &aixle_spi_gpio_cs,
	.bus_num	= 2,
	.num_chipselect = 2,
};


static struct platform_device aixle_gpio_spi_device = {
	.name	= "spi_at91_gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &spi_gpio_cfg,
	},
};


static void __init aixle_add_device_bitbang_spi(struct spi_board_info *devices, int nr_devices)
{
	short i;
	short enable_spi = 0;
	
	/* Add SPI devices controlled by this master. */
	for (i = 0; i < nr_devices; i++) {
		if ( devices[i].bus_num == spi_gpio_cfg.bus_num )
		{
			printk(KERN_INFO "Atmel GPIO SPI: Add device (%s)\n",
			       devices[i].modalias);
			spi_register_board_info(&devices[i], 1);
			enable_spi = 1;
		}
	}
	
	if (enable_spi == 1) {
		printk(KERN_INFO "Atmel GPIO SPI: Enable bus %d\n",
		       spi_gpio_cfg.bus_num);
		
		// Configure Chip Select signals
		at91_set_gpio_output(AT91_PIN_PB15, 1);
		at91_set_gpio_output(AT91_PIN_PB16, 1);

		// SPI clock, MOSI
		at91_set_gpio_output(spi_gpio_cfg.pin_clk, 0);
		at91_set_gpio_output(spi_gpio_cfg.pin_mosi, 0);

		// SPI MISO (enable pullup)
		at91_set_gpio_input(spi_gpio_cfg.pin_miso, 1);
		
		platform_device_register(&aixle_gpio_spi_device);
	}
	else {
		printk(KERN_INFO "Atmel GPIO SPI: No devices on bus %d: bus "
				 "disabled\n", spi_gpio_cfg.bus_num);
		return;	
	}
}

#else
static void __init aixle_add_device_bitbang_spi(struct spi_board_info *devices, int nr_devices){}
#endif


/*
 * MCI (SD/MMC)
 */
static struct at91_mmc_data __initdata aixle_mmc_data = {
	.wire4		= 1,
	.det_pin	= AT91_PIN_PE18,
	.wp_pin		= AT91_PIN_PE19,
//	.vcc_pin	= ... not connected
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata aixle_macb_data = {
	.phy_irq_pin	= AT91_PIN_PE31,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata aixle_nand_partition[] = {
	{
		.name	= "Linux Kernel",
		.offset	= 0,
		.size	= SZ_16M,
	},
	{
		.name	= "Root Filesystem",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 240 * SZ_1M,
	}
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(aixle_nand_partition);
	return aixle_nand_partition;
}

static struct atmel_nand_data __initdata aixle_nand_data = {
	.ale		= 21,
	.cle		= 22,
//	.det_pin	= ... not connected
	.rdy_pin	= AT91_PIN_PA22,
	.enable_pin	= AT91_PIN_PD15,
	.partition_info	= nand_partitions,
};

static struct sam9_smc_config __initdata aixle_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 1,
	.ncs_write_setup	= 0,
	.nwe_setup		= 1,

	.ncs_read_pulse		= 3,
	.nrd_pulse		= 3,
	.ncs_write_pulse	= 3,
	.nwe_pulse		= 3,

	.read_cycle		= 5,
	.write_cycle		= 5,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE |
				  AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_DBW_8,
	.tdf_cycles		= 2,
};

static void __init aixle_add_device_nand(void)
{
	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &aixle_nand_smc_config);

	at91_add_device_nand(&aixle_nand_data);
}


/*
 * LCD Controller
 */
#if defined(CONFIG_FB_ATMEL) || defined(CONFIG_FB_ATMEL_MODULE)
static struct fb_videomode at91_tft_vga_modes[] = {
{
	.name		= "800x600@70",
	.refresh	= 70,
	.xres		= 800,
	.yres		= 600,
	/* picoseconds (1000000 / pixelclock) */
	.pixclock	= 22200, // 45 MHz
	.left_margin	= 75,
	.right_margin	= 75,
	.upper_margin	= 10,
	.lower_margin	= 10,
	.hsync_len	= 32,
	.vsync_len	= 32,

	.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode		= FB_VMODE_NONINTERLACED,
},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "lcd", // 4 char array
	.monitor        = "TFT Panel",
	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 640000,
	.vfmin		= 60,
	.vfmax		= 80,
};

#define AT91SAM9263_DEFAULT_LCDCON2 	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT    \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE)

static void at91_lcdc_power_control(int on)
{
	/* PD3 enables the LVDS driver */
	if (on)
		at91_set_gpio_output(AT91_PIN_PD3, 1);
	else
		at91_set_gpio_output(AT91_PIN_PD3, 0);
}

/* Driver data */
static struct atmel_lcdfb_info __initdata aixle_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.default_bpp			= 16,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9263_DEFAULT_LCDCON2,
	.default_monspecs		= &at91fb_default_monspecs,
	.atmel_lcdfb_power_control	= at91_lcdc_power_control,
	.guard_time			= 1,
};

#else
static struct atmel_lcdfb_info __initdata aixle_lcdc_data;
#endif


#if defined(CONFIG_BACKLIGHT_GPIO) || defined(CONFIG_BACKLIGHT_GPIO_MODULE)
static void aixle_set_backlight(int value)
{
	// Backlight pin is AT91_PIN_PE12

	if (value > 0) {
		//backlight on
		at91_set_gpio_value(AT91_PIN_PE12, 1);
	} else {
		// backlight off
		at91_set_gpio_value(AT91_PIN_PE12, 0);
	}
}

static struct gpio_backlight_data aixle_bl_data = {
	.set_backlight	= &aixle_set_backlight
};

static struct platform_device aixle_gpio_bl_device = {
	.name   = "gpio-backlight",
	.id = -1,
	.dev    = {
		.platform_data  = &aixle_bl_data,
	},
};

static void __init aixle_add_backlight(void)
{
	platform_device_register(&aixle_gpio_bl_device);
}
#else
static void __init aixle_add_backlight(void) {}
#endif


/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button aixle_buttons[] = {
	{	/* On board push button */
		.code		= KEY_ENTER,
		.gpio		= AT91_PIN_PB10,
		.active_low	= 1,
		.desc		= "user_pb",
		.wakeup		= 1,
		.debounce_interval	= 40,
	},
	{	/* PWR Button */
		.code		= KEY_POWER,
		.gpio		= AT91_PIN_PB28,
		.active_low	= 1,
		.desc		= "Power Button",
		.wakeup		= 1,
		.debounce_interval	= 40,
	},
	{	/* VOL+ Button */
		.code		= KEY_VOLUMEUP,
		.gpio		= AT91_PIN_PB29,
		.active_low	= 1,
		.desc		= "Volume+ Button",
		.debounce_interval	= 40,
	},
	{	/* Vol- Button */
		.code		= KEY_VOLUMEDOWN,
		.gpio		= AT91_PIN_PB30,
		.active_low	= 1,
		.desc		= "Volume- Button",
		.debounce_interval	= 40,
	},
	{	/* MENU Button */
		.code		= KEY_MENU,
		.gpio		= AT91_PIN_PB25,
		.active_low	= 1,
		.desc		= "Menu Button",
		.debounce_interval	= 40,
	},
	{	/* CH+ Button */
		.code		= KEY_CHANNELUP,
		.gpio		= AT91_PIN_PB27,
		.active_low	= 1,
		.desc		= "Channel+ Button",
		.debounce_interval	= 40,
	},
	{	/* CH- Button */
		.code		= KEY_CHANNELDOWN,
		.gpio		= AT91_PIN_PB26,
		.active_low	= 1,
		.desc		= "Channel- Button",
		.debounce_interval	= 40,
	},
	{	/* AV Button */
		.code		= KEY_VIDEO,
		.gpio		= AT91_PIN_PB31,
		.active_low	= 1,
		.desc		= "AV Button",
		.debounce_interval	= 40,
	},
};

static struct gpio_keys_platform_data aixle_button_data = {
	.buttons	= aixle_buttons,
	.nbuttons	= ARRAY_SIZE(aixle_buttons),
};

static struct platform_device aixle_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &aixle_button_data,
	}
};

static void __init aixle_add_device_buttons(void)
{
	/* on board push button, pull up enabled */
	at91_set_gpio_input(AT91_PIN_PB10, 1);
	at91_set_deglitch(AT91_PIN_PB10, 1);
	
	at91_set_gpio_input(AT91_PIN_PB25, 1);
	at91_set_deglitch(AT91_PIN_PB25, 1);
	
	at91_set_gpio_input(AT91_PIN_PB26, 1);
	at91_set_deglitch(AT91_PIN_PB26, 1);
	
	at91_set_gpio_input(AT91_PIN_PB27, 1);
	at91_set_deglitch(AT91_PIN_PB27, 1);
	
	at91_set_gpio_input(AT91_PIN_PB28, 1);
	at91_set_deglitch(AT91_PIN_PB28, 1);
	
	at91_set_gpio_input(AT91_PIN_PB29, 1);
	at91_set_deglitch(AT91_PIN_PB29, 1);
	
	at91_set_gpio_input(AT91_PIN_PB30, 1);
	at91_set_deglitch(AT91_PIN_PB30, 1);
	
	at91_set_gpio_input(AT91_PIN_PB31, 1);
	at91_set_deglitch(AT91_PIN_PB31, 1);

	platform_device_register(&aixle_button_device);
}
#else
static void __init aixle_add_device_buttons(void) {}
#endif


/*
 * LEDs
 */
static struct gpio_led aixle_leds[] = {
	{	/* AIXLE on-board LED */
		.name			= "aixle",
		.gpio			= AT91_PIN_PB20,
		.active_low		= 1,
		.default_trigger	= "nand-disk",
	},
	{	/* Power LED */
		.name			= "power",
		.gpio			= AT91_PIN_PB18,
		.active_low		= 1,
		.default_trigger	= "default-on",
	},
};


/*
 * AC97
 */
static struct atmel_ac97_data aixle_ac97_data = {
	.reset_pin	= AT91_PIN_PB6,
};


#if defined(CONFIG_SND_AT91_SOC_AC97_GENERIC) ||\
    defined(CONFIG_SND_AT91_SOC_AC97_GENERIC_MODULE)
static struct platform_device aixle_ac97_codec = {
	.name		= "at91-ac97-generic",
	.id		= 1,
	.num_resources	= 0,
};


void __init aixle_add_device_ac97_codec(void)
{
	printk(KERN_INFO "AT91: Atmel AC97 codec\n");
	platform_device_register(&aixle_ac97_codec);
}
#else
void __init aixle_add_device_ac97_codec(void){}
#endif


static void __init aixle_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&aixle_usbh_data);
	/* USB Device */
	at91_add_device_udc(&aixle_udc_data);
	/* SPI */
	at91_add_device_spi(aixle_spi_devices, ARRAY_SIZE(aixle_spi_devices));
	/* bitbang SPI */
	aixle_add_device_bitbang_spi(aixle_spi_devices, ARRAY_SIZE(aixle_spi_devices));
	/* Touchscreen */
	aixle_add_device_ts();
	/* nRF (not written yet) */
	aixle_add_device_nrf();
	/* MMC */
	at91_add_device_mmc(1, &aixle_mmc_data);
	/* Ethernet */
	at91_add_device_eth(&aixle_macb_data);
	/* NAND */
	aixle_add_device_nand();
	/* I2C */
	at91_add_device_i2c(NULL, 0);
	/* GPIO controlled backlight */
	aixle_add_backlight();
	/* LCD Controller */
	at91_add_device_lcdc(&aixle_lcdc_data);
	/* Push Buttons */
	aixle_add_device_buttons();
	/* AC97 */
	at91_add_device_ac97(&aixle_ac97_data);
	aixle_add_device_ac97_codec();
	/* LEDs */
	at91_gpio_leds(aixle_leds, ARRAY_SIZE(aixle_leds));
	/* shutdown controller, wakeup button (5 msec low) */
	at91_sys_write(AT91_SHDW_MR, AT91_SHDW_CPTWK0_(10) | AT91_SHDW_WKMODE0_LOW
				| AT91_SHDW_RTTWKEN);
}

MACHINE_START(AIXLE, "4D Electronics AIXLE")
	/* Maintainer: 4D Electronics */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= aixle_map_io,
	.init_irq	= aixle_init_irq,
	.init_machine	= aixle_board_init,
MACHINE_END
