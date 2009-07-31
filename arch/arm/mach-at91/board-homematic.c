/*
 * linux/arch/arm/mach-at91/board-homematic.c
 *
 *  Copyright (C) 2007 eQ-3 Entwicklung GmbH
 *
 *  based on work
 *	Copyright (C) 2005 SAN People
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
#include <linux/mtd/physmap.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91rm9200_mc.h>

#include "generic.h"


/*
 * Serial port configuration.
 *    0 .. 3 = USART0 .. USART3
 *    4      = DBGU
 */
static struct at91_uart_config __initdata homematic_uart_config = {
	.console_tty	= 0,				/* ttyS0 */
	.nr_tty		= 2,
	.tty_map	= { 4, 1, -1, -1, -1 }		/* ttyS0, ..., ttyS4 */
};

static void __init homematic_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91rm9200_initialize(18432000, AT91RM9200_BGA);

	/* Setup the serial ports and console */
	at91_init_serial(&homematic_uart_config);
}

static void __init homematic_init_irq(void)
{
	at91rm9200_init_interrupts(NULL);
}

static struct at91_eth_data __initdata homematic_eth_data = {
	.phy_irq_pin	= AT91_PIN_PC4,
	.is_rmii	= 0,
};

static struct at91_usbh_data __initdata homematic_usbh_data = {
	.ports		= 2,
};

static struct at91_udc_data __initdata homematic_udc_data = {
	.vbus_pin	= AT91_PIN_PD4,
	.pullup_pin	= AT91_PIN_PD5,
};

static struct at91_mmc_data __initdata homematic_mmc_data = {
	.slot_b		= 0,
	.wire4		= 1,
};

static struct spi_board_info homematic_spi_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
	},
};

static struct mtd_partition __initdata homematic_nand_partition[] = {
	{
		.name	= "root",
		.offset	= 0,
		.size   = SZ_32M,
	}, {
		.name	= "storage",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(homematic_nand_partition);
	return homematic_nand_partition;
}

static struct atmel_nand_data __initdata homematic_nand_data = {
	.ale		= 22,
	.cle		= 21,
//	.det_pin	= AT91_PIN_PB1,
	.rdy_pin	= AT91_PIN_PC2,
	.enable_pin	= AT91_PIN_PC0,
	.partition_info	= nand_partitions,
};

static void __init homematic_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&homematic_eth_data);
	/* USB Host */
	at91_add_device_usbh(&homematic_usbh_data);
	/* USB Device */
	at91_add_device_udc(&homematic_udc_data);
	at91_set_multi_drive(homematic_udc_data.pullup_pin, 1);	/* pullup_pin is connected to reset */
	/* I2C */
	at91_add_device_i2c(NULL, 0);
	/* SPI */
	at91_add_device_spi(homematic_spi_devices, ARRAY_SIZE(homematic_spi_devices));
#ifdef CONFIG_MTD_AT91_DATAFLASH_CARD
	/* DataFlash card */
	at91_set_gpio_output(AT91_PIN_PB7, 0);
#else
	/* MMC */
	at91_set_gpio_output(AT91_PIN_PB7, 1);	/* this MMC card slot can optionally use SPI signaling (CS3). */
	at91_add_device_mmc(0, &homematic_mmc_data);
#endif
	/* NAND */
	at91_add_device_nand(&homematic_nand_data);
}

MACHINE_START(HOMEMATIC, "HomeMatic")
	/* Maintainer: eQ-3 */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91rm9200_timer,
	.map_io		= homematic_map_io,
	.init_irq	= homematic_init_irq,
	.init_machine	= homematic_board_init,
MACHINE_END
