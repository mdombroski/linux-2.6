/* linux/include/asm-arm/arch-at91/spi-gpio.h
 *
 * AT91 - GPIO SPI Controller platfrom_device info
 *
 * Copyright (c) 2008 Matthew Dombroski
 * Copyright (c) 2008 4D Electronics
 *
 * Based on work by:
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_SPIGPIO_H
#define __ASM_ARCH_SPIGPIO_H __FILE__

struct at91_spigpio_info {
	unsigned long		 pin_clk;
	unsigned long		 pin_mosi;
	unsigned long		 pin_miso;

	int bus_num;
	int num_chipselect;

	void (*chip_select)(struct at91_spigpio_info *spi, int cs, int value);
};


#endif /* __ASM_ARCH_SPIGPIO_H */
