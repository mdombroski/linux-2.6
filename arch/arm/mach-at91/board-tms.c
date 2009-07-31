/*
* linux/arch/arm/mach-at91/board-tms.c
*
*  Copyright (C) 2005 SAN People
*
*  Adapted from board-dk to sweda TMS-100 by Luiz de Barros <lboneto@gmail.com>
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
#include <linux/serial_8250.h>


#define SERIAL_FLAGS	(UPF_BOOT_AUTOCONF | UPF_IOREMAP| UPF_SHARE_IRQ)
#define SERIAL_CLK	(1843200)


/*---------------------------------------------------------------------
 * External UART
 */

#define PORT(_base, _irq)				\
	{						\
		.mapbase	= _base,		\
		.irq		= _irq,			\
		.uartclk	= SERIAL_CLK,		\
		.iotype		= UPIO_MEM,		\
		.regshift	= 0,			\
		.flags		= SERIAL_FLAGS,		\
	}

static struct plat_serial8250_port tms_data[] = {
	PORT(AT91_CHIPSELECT_6, AT91_PIN_PC3),
	PORT(AT91_CHIPSELECT_7, AT91_PIN_PC5),
	{ },
};

static struct platform_device tms_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			=
		{
			.platform_data	= &tms_data,
		},
};

static void setup_external_uart(void)
{
	at91_sys_write(AT91_SMC_CSR(2),
				  AT91_SMC_ACSS_STD
				| AT91_SMC_DBW_8
				| AT91_SMC_BAT
				| AT91_SMC_WSEN
				| AT91_SMC_NWS_(32)	/* wait states */
				| AT91_SMC_RWSETUP_(6)	/* setup time */
				| AT91_SMC_RWHOLD_(4)	/* hold time */

	);
	at91_sys_write(AT91_SMC_CSR(6),
				  AT91_SMC_ACSS_STD
				| AT91_SMC_DBW_8
				| AT91_SMC_BAT
				| AT91_SMC_WSEN
				| AT91_SMC_NWS_(32)	/* wait states */
				| AT91_SMC_RWSETUP_(6)	/* setup time */
				| AT91_SMC_RWHOLD_(4)	/* hold time */

	);
	at91_sys_write(AT91_SMC_CSR(7),
				  AT91_SMC_ACSS_STD
				| AT91_SMC_DBW_8
				| AT91_SMC_BAT
				| AT91_SMC_WSEN
				| AT91_SMC_NWS_(32)	/* wait states */
				| AT91_SMC_RWSETUP_(6)	/* setup time */
				| AT91_SMC_RWHOLD_(4)	/* hold time */
	);

	platform_device_register(&tms_device);
}


/*
 * Serial port configuration.
 *    0 .. 3 = USART0 .. USART3
 *    4      = DBGU
 */
static struct at91_uart_config __initdata tms_uart_config = {
	.console_tty	= 0,				/* ttyS0 */
	.nr_tty		= 5,
	.tty_map	= { 4, 0, 1, 2, 3 }		/* ttyS0, ..., ttyS4 */
};

static void __init tms_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91rm9200_initialize(18432000, AT91RM9200_BGA);

	/* Setup the LEDs */
	at91_init_leds(AT91_PIN_PB2, AT91_PIN_PB2);

	/* Setup the serial ports and console */
	at91_init_serial(&tms_uart_config);
}

static void __init tms_init_irq(void)
{
	at91rm9200_init_interrupts(NULL);
}


static struct at91_eth_data __initdata tms_eth_data = {
	.phy_irq_pin	= AT91_PIN_PC4,
	.is_rmii	= 1,
};

static struct at91_usbh_data __initdata tms_usbh_data = {
	.ports		= 2,
};

static struct spi_board_info tms_spi_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
	},
	{	 /* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 1,
		.max_speed_hz	= 15 * 1000 * 1000,
	}
};

static struct i2c_board_info __initdata tms_i2c_devices[] = {
	{
		I2C_BOARD_INFO("isl1208", 0x6f),
	}
};

static void __init tms_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* Ethernet */
	at91_add_device_eth(&tms_eth_data);
	at91_add_device_usbh(&tms_usbh_data);
	/* I2C */
	at91_add_device_i2c(tms_i2c_devices, ARRAY_SIZE(tms_i2c_devices));
	/* SPI */
	at91_add_device_spi(tms_spi_devices, ARRAY_SIZE(tms_spi_devices));
	/* Two port external UART */
	setup_external_uart();
}

MACHINE_START(SWEDATMS, "Sweda TMS-100 Board")
	/* Maintainer: Luiz de Barros */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91rm9200_timer,
	.map_io		= tms_map_io,
	.init_irq	= tms_init_irq,
	.init_machine	= tms_board_init,
MACHINE_END
