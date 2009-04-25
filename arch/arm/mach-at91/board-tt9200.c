/*
 * linux/arch/arm/mach-at91rm9200/board-tt9200.c
 * Copyright (C) 2007 Toptechnology
 *
 * Based on board-ecbat91.c
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
#include <linux/spi/flash.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>

#include "generic.h"


static void __init tt9200_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91rm9200_initialize(18432000, AT91RM9200_PQFP);

	/* Setup the LEDs */
	at91_init_leds(AT91_PIN_PB27, AT91_PIN_PB27);

	/* DBGU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 on ttyS1. (Rx & Tx only) */
	at91_register_uart(AT91RM9200_ID_US0, 1, 0);

	/* USART1 on ttyS2. (Rx & Tx only) */
	at91_register_uart(AT91RM9200_ID_US1, 2, 0);

	/* USART2 on ttyS3. (Rx & Tx only) */
	at91_register_uart(AT91RM9200_ID_US2, 3, 0);

	/* USART3 on ttyS4. (Rx, Tx, CTS, RTS) */
	at91_register_uart(AT91RM9200_ID_US3, 4, ATMEL_UART_CTS | ATMEL_UART_RTS);

	/* Console on ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init tt9200_init_irq(void)
{
	at91rm9200_init_interrupts(NULL);
}

static struct at91_eth_data __initdata tt9200_eth_data = {
	.phy_irq_pin	= AT91_PIN_PB29,
	.is_rmii	= 0,
};

static struct at91_usbh_data __initdata tt9200_usbh_data = {
	.ports		= 1,
};

static struct i2c_board_info __initdata tt9200_i2c_devices[] = {
	{
		I2C_BOARD_INFO("m41t80", 0x68),
	}
};

static struct at91_mmc_data __initdata tt9200_mmc_data = {
	.slot_b		= 0,
	.wire4		= 1,
};


#if defined(CONFIG_MTD_DATAFLASH)
static struct mtd_partition __initdata tt9200_flash_partitions[] =
{
	{
		.name	= "Darrell",
		.offset	= 0,
		.size	= 12 * 1056,
	},
	{
		.name	= "U-boot",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 110 * 1056,
	},
	{
		.name	= "U-boot env",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 8 * 1056,
	},
	{
		.name	= "Kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 1534 * 1056,
	},
	{
		.name	= "Filesystem",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	}
};

static struct flash_platform_data __initdata tt9200_flash_platform = {
	.name		= "SPI Dataflash",
	.parts		= tt9200_flash_partitions,
	.nr_parts	= ARRAY_SIZE(tt9200_flash_partitions)
};

#endif

static struct spi_board_info __initdata tt9200_spi_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		/* Errata #13 */
		.max_speed_hz	= 5 * 1000 * 1000,
		.bus_num	= 0,
#if defined(CONFIG_MTD_DATAFLASH)
		.platform_data	= &tt9200_flash_platform,
#endif
	},
};

static struct gpio_led tt9200_leds[] = {
	{
		.name			= "led0",
		.gpio			= AT91_PIN_PB27,
		.active_low		= 1,
		.default_trigger	= "heartbeat",
	}
};

static void __init tt9200_board_init(void)
{
	/* Serial */
	at91_add_device_serial();

	/* Ethernet */
	at91_add_device_eth(&tt9200_eth_data);

	/* USB Host */
	at91_add_device_usbh(&tt9200_usbh_data);

	/* I2C */
	at91_add_device_i2c(tt9200_i2c_devices, ARRAY_SIZE(tt9200_i2c_devices));

	/* MMC */
	at91_add_device_mmc(0, &tt9200_mmc_data);

	/* LEDS */
	at91_gpio_leds(tt9200_leds, ARRAY_SIZE(tt9200_leds));

	/* SPI */
	at91_add_device_spi(tt9200_spi_devices, ARRAY_SIZE(tt9200_spi_devices));
}

MACHINE_START(TT9200, "Toptech TT9200")
	/* Maintainer: toptech.com.ar */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91rm9200_timer,
	.map_io		= tt9200_map_io,
	.init_irq	= tt9200_init_irq,
	.init_machine	= tt9200_board_init,
MACHINE_END
