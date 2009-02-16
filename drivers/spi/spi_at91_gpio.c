/* linux/drivers/spi/spi_at91_bitbang.c
 *
 * Atmel AT91 GPIO based SPI driver
 *
 * Copyright (c) 2008 Matthew Dombroski
 * Copyright (c) 2008 4D Electronics
 *
 * Based on linux/drivers/spi/spi_s3c24xx.c
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <mach/gpio.h>
#include <mach/spi-gpio.h>
#include <mach/hardware.h>

struct at91_spigpio {
	struct spi_bitbang bitbang;

	struct at91_spigpio_info *info;
	struct platform_device *dev;
};

static inline struct at91_spigpio *spidev_to_sg(struct spi_device *spi)
{
	return spi->controller_data;
}

static inline void setsck(struct spi_device *dev, int on)
{
	struct at91_spigpio *sg = spi_master_get_devdata(dev->master);
	at91_set_gpio_output(sg->info->pin_clk, on ? 1 : 0);
}

static inline void setmosi(struct spi_device *dev, int on)
{
	struct at91_spigpio *sg = spi_master_get_devdata(dev->master);
	at91_set_gpio_output(sg->info->pin_mosi, on ? 1 : 0);
}

static inline u32 getmiso(struct spi_device *dev)
{
	struct at91_spigpio *sg = spi_master_get_devdata(dev->master);
	return at91_get_gpio_value(sg->info->pin_miso) ? 1 : 0;
}

#define spidelay(x) ndelay(x)

#define	EXPAND_BITBANG_TXRX
#if defined ( EXPAND_BITBANG_TXRX )
#include <linux/spi/spi_bitbang.h>

static u32 at91_spigpio_txrx_mode0(struct spi_device *spi,
				   unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 at91_spigpio_txrx_mode1(struct spi_device *spi,
				   unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 at91_spigpio_txrx_mode2(struct spi_device *spi,
				   unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static u32 at91_spigpio_txrx_mode3(struct spi_device *spi,
				   unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, bits);
}
#endif

static void at91_spigpio_chipselect(struct spi_device *dev, int value)
{
	struct at91_spigpio *sg = spi_master_get_devdata(dev->master);

	if (sg->info && sg->info->chip_select)
		(sg->info->chip_select) (sg->info, dev->chip_select, value);
}

static int at91_spigpio_probe(struct platform_device *dev)
{
	struct spi_master *master;
	struct at91_spigpio *sp;
	int ret;

	master = spi_alloc_master(&dev->dev, sizeof(struct at91_spigpio));
	if (master == NULL) {
		dev_err(&dev->dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err;
	}

	sp = spi_master_get_devdata(master);

	platform_set_drvdata(dev, sp);

	/* copy in the plkatform data */
	sp->info = dev->dev.platform_data;

	/* setup spi bitbang adaptor */
	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.master->bus_num = sp->info->bus_num;
	sp->bitbang.master->num_chipselect = sp->info->num_chipselect;
	sp->bitbang.chipselect = at91_spigpio_chipselect;

	sp->bitbang.txrx_word[SPI_MODE_0] = at91_spigpio_txrx_mode0;
	sp->bitbang.txrx_word[SPI_MODE_1] = at91_spigpio_txrx_mode1;
	sp->bitbang.txrx_word[SPI_MODE_2] = at91_spigpio_txrx_mode2;
	sp->bitbang.txrx_word[SPI_MODE_3] = at91_spigpio_txrx_mode3;

	ret = spi_bitbang_start(&sp->bitbang);
	if (ret)
		goto err_no_bitbang;

	return 0;

 err_no_bitbang:
	spi_master_put(sp->bitbang.master);
 err:
	return ret;

}

static int at91_spigpio_remove(struct platform_device *dev)
{
	struct at91_spigpio *sp = platform_get_drvdata(dev);

	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);

	return 0;
}

/* all gpio should be held over suspend/resume, so we should */
/* not need to deal with this */
#define at91_spigpio_suspend NULL
#define at91_spigpio_resume NULL

static struct platform_driver at91_spigpio_drv = {
	.probe = at91_spigpio_probe,
	.remove = at91_spigpio_remove,
	.suspend = at91_spigpio_suspend,
	.resume = at91_spigpio_resume,
	.driver = {
		   .name = "spi_at91_gpio",
		   .owner = THIS_MODULE,
		   },
};

static int __init at91_spigpio_init(void)
{
	return platform_driver_register(&at91_spigpio_drv);
}

static void __exit at91_spigpio_exit(void)
{
	platform_driver_unregister(&at91_spigpio_drv);
}

module_init(at91_spigpio_init);
module_exit(at91_spigpio_exit);

MODULE_ALIAS("platform:spi_at91_gpio");
MODULE_DESCRIPTION("AT91 SPI Driver");
MODULE_AUTHOR("Matthew Dombroski, <mdombroski@4d-electronics.co.nz>");
MODULE_LICENSE("GPL");
