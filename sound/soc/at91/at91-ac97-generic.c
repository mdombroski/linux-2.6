/*
 * at91-ac7-generic.c  --  Generic AC97 codec attached to AT91
 *
 * Copyright (C) 2008 4D Electronics
 *	http://www.4d-electronics.co.nz
 *	Matthew Dombroski <mdombroski@4d-electronics.co.nz>
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
 *
 *  Revision history
 *    17 July 2008	Initial version.
 *    04 March 2009	Port to linux 2.6.28
 *    05 July 2009 Port to linux 2.6.30
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/board.h>

#include "../codecs/ac97.h"
#include "at91-pcm.h"
#include "at91-ac97.h"


static int machine_init(struct snd_soc_codec *codec)
{
	snd_soc_dapm_sync(codec);
	return 0;
}

static struct snd_soc_dai_link at91_ac97_generic_dai[] = {
{
	.name		= "AC97",
	.stream_name	= "AC97 HiFi",
	.cpu_dai	= &at91_ac97_dai[0],
	.codec_dai	= &ac97_dai,
	.init		= machine_init,
	.ops		= NULL,
},
};

static struct snd_soc_card at91_ac97_generic_card = {
	.name		= "Atmel AT91 AC97",
	.platform	= &at91_soc_platform,
	.dai_link	= at91_ac97_generic_dai,
	.num_links	= 1,
};

static struct snd_soc_device at91_ac97_generic_devdata = {
	.card		= &at91_ac97_generic_card,
	.codec_dev	= &soc_codec_dev_ac97,
};

static struct platform_device *at91_ac97_generic_device;

static int at91_ac97_generic_probe(struct platform_device *pdev)
{
	int ret = 0;
	
	printk(KERN_INFO "at91_ac97_generic probe\n");

	at91_ac97_generic_device = platform_device_alloc("soc-audio", -1);
	
	if (!at91_ac97_generic_device)
	{
		printk(KERN_WARNING "at91_ac97_generic probe failed could not allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(at91_ac97_generic_device, &at91_ac97_generic_devdata);
	at91_ac97_generic_devdata.dev = &at91_ac97_generic_device->dev;

	ret = platform_device_add(at91_ac97_generic_device);
	
	if (ret)
	{
		printk(KERN_WARNING "at91_ac97_generic probe failed to register soc-audio\n");
		platform_device_put(at91_ac97_generic_device);
	}
	
	return ret;
}

static int at91_ac97_generic_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "at91_ac97_generic remove\n");
	
	platform_device_unregister(at91_ac97_generic_device);
	return 0;
}

static struct platform_driver at91_ac97_generic_driver =
{
	.probe	= at91_ac97_generic_probe,
	.remove	= at91_ac97_generic_remove,
	.driver	= {
		.name = "at91-ac97-generic",
	},
};

static int __init at91_ac97_generic_init(void)
{
	return platform_driver_register(&at91_ac97_generic_driver);
}

static void __exit at91_ac97_generic_exit(void)
{
	platform_driver_unregister(&at91_ac97_generic_driver);
}

module_init(at91_ac97_generic_init);
module_exit(at91_ac97_generic_exit);

/* Module information */
MODULE_ALIAS("platform:at91-ac97-generic");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ALSA SoC AT91 with AC97 Codec");
MODULE_AUTHOR("Matthew Dombroski <mdombroski@4d-electronics.co.nz>");
