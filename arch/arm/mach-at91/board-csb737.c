/*
 * linux/arch/arm/mach-at91/board-csb737.c
 *
 *  Copyright (C) 2008 Bill Gatliff
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>

#include "sam9_smc.h"
#include "generic.h"


static void __init csb_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9263_initialize(18432000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init csb_init_irq(void)
{
	at91sam9263_init_interrupts(NULL);
}


/*
 * Ethernet
 */
static struct at91_eth_data __initdata csb_macb_data = {
	.phy_irq_pin	= AT91_PIN_PE31,
	.is_rmii	= 1,
};


/*
 * NOR Flash
 */
#define CSB_FLASH_BASE	AT91_CHIPSELECT_0
#define CSB_FLASH_SIZE	SZ_64M

static struct mtd_partition csb_flash_partitions[] = {
	{
		.name		= "uMON tfs",
		.offset		= 0,
		.size		= SZ_8M,
		.mask_flags	= MTD_WRITEABLE,	/* read only */
	},
	{
		.name		= "nor0",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= MTDPART_SIZ_FULL,
	}
};

static struct physmap_flash_data csb_flash_data = {
	.width		= 2,
	.parts		= csb_flash_partitions,
	.nr_parts	= ARRAY_SIZE(csb_flash_partitions),
};

static struct resource csb_flash_resources[] = {
	{
		.start	= CSB_FLASH_BASE,
		.end	= CSB_FLASH_BASE + CSB_FLASH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device csb_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
				.platform_data = &csb_flash_data,
			},
	.resource	= csb_flash_resources,
	.num_resources	= ARRAY_SIZE(csb_flash_resources),
};


/*
 * NAND Flash
 */
static struct mtd_partition __initdata csb_nand_partitions[] = {
	{
		.name	= "nand0",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(csb_nand_partitions);
	return csb_nand_partitions;
}

static struct atmel_nand_data __initdata csb_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PA22,
	.enable_pin	= AT91_PIN_PD15,
	.partition_info	= nand_partitions,
};

static struct sam9_smc_config __initdata csb_nand_smc_config = {
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

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_DBW_8,
	.tdf_cycles		= 2,
};

static void __init csb_add_device_nand(void)
{
	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &csb_nand_smc_config);

	at91_add_device_nand(&csb_nand_data);
}


/*
 * LEDs
 */
static struct gpio_led csb_leds[] = {
	{
		.name			= "gpio0",
		.gpio			= AT91_PIN_PD4,
		.active_low		= 1,
		.default_trigger	= "heartbeat",
	},
};


static void __init csb_board_init(void)
{
	at91_gpio_leds(csb_leds, ARRAY_SIZE(csb_leds));
	at91_add_device_serial();
	at91_add_device_eth(&csb_macb_data);
	csb_add_device_nand();
	platform_device_register(&csb_flash);
}

MACHINE_START(CSB737, "Cogent CSB737 SOM (AT91SAM9263)")
	/* Maintainer: Bill Gatliff <bgat@billgatliff.com> */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= csb_map_io,
	.init_irq	= csb_init_irq,
	.init_machine	= csb_board_init,
MACHINE_END
