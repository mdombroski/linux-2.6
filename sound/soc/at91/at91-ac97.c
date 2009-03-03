/*
 * sound/soc/at91/at91-ac97.c  --  ALSA Soc Audio Layer (platform)
 *
 * Copyright (C) 2008 4D Electronics
 *	http://www.4d-electronics.co.nz
 *	Matthew Dombroski <mdombroski@4d-electronics.co.nz>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    17 July 2008   Initial version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/atmel_pdc.h>
#include <linux/mutex.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <mach/gpio.h>
#include <mach/at91_ac97.h>
#include <mach/board.h>
#include <mach/hardware.h>

#include "at91-ac97.h"
#include "at91-pcm.h"

#define CODEC_TIMEOUT 0x100

#define AT91_AC97_DEBUG 0
#if AT91_AC97_DEBUG
#  define DBG(x...) printk(KERN_WARNING "at91-ac97: " x)
#else
#  define DBG(x...)
#endif


static DECLARE_MUTEX(ac97_mutex);
DECLARE_COMPLETION(ac97_codec_completion);


/*
 * AC97 PDC registers required by the PCM DMA engine.
 */
static struct at91_pdc_regs pdc_tx_reg = {
	.xpr		= ATMEL_PDC_TPR,
	.xcr		= ATMEL_PDC_TCR,
	.xnpr		= ATMEL_PDC_TNPR,
	.xncr		= ATMEL_PDC_TNCR,
	.ptcr		= ATMEL_PDC_PTCR,
};

static struct at91_pdc_regs pdc_rx_reg = {
	.xpr		= ATMEL_PDC_RPR,
	.xcr		= ATMEL_PDC_RCR,
	.xnpr		= ATMEL_PDC_RNPR,
	.xncr		= ATMEL_PDC_RNCR,
	.ptcr		= ATMEL_PDC_PTCR,
};


static struct at91_pcm_dma_params at91_ac97_dma_params[2] =
{
	{
		.name			= "AC97 PCM out",
		.pdc			= &pdc_tx_reg,
		.dma_end		= AT91C_AC97C_ENDTX,
		.dma_endbuf		= AT91C_AC97C_TXBUFE,
		.dma_enable		= ATMEL_PDC_TXTEN,
		.dma_disable		= ATMEL_PDC_TXTDIS,
		.dma_enqueue		= NULL,
	},
	{
		.name			= "AC97 PCM in",
		.pdc			= &pdc_rx_reg,
		.dma_end		= AT91C_AC97C_ENDRX,
		.dma_endbuf		= AT91C_AC97C_RXBUFF,
		.dma_enable		= ATMEL_PDC_RXTEN,
		.dma_disable		= ATMEL_PDC_RXTDIS,
		.dma_enqueue		= NULL,
	},
};


struct at91_ac97_info {
	spinlock_t lock;
	void __iomem *regs;
	struct clk *ac97_clk;
	u8 reset_pin;
	int irq;
	
	/* Stream direction mask, 1=playback, 2=capture */
	/* Only allow 1 stream in each direction */
	unsigned short dir_mask;
	
	/* The current pcm dma operations */
	struct at91_pcm_dma_params *dma_params[2];
};


static struct at91_ac97_info at91_ac97_private = {
	.lock = __SPIN_LOCK_UNLOCKED(at91_ac97_private.lock),
};



void at91_ac97_drive_reset(struct at91_ac97_info *chip, unsigned int value)
{
	if ( chip->reset_pin ){
		at91_set_gpio_value(chip->reset_pin, value);
	}
}


/* Read data from the AC97 bus directly */
static unsigned short at91_ac97_read(struct snd_ac97 *ac97, unsigned short reg)
{
	u32 word;
	int timeout = CODEC_TIMEOUT;
	u16 ret = 0xffff;

	down(&ac97_mutex);
	
	do
	{
		word = readl( (&at91_ac97_private)->regs + AC97C_COSR );
		if( word & AT91C_AC97C_TXRDY )
		{
			word = (0x80 | (reg & 0x7f)) << 16;
			writel( word, (&at91_ac97_private)->regs + AC97C_COTHR );
			break;
		}
		udelay(1);
	} while (--timeout);

	if (!timeout)
		goto timed_out;

	timeout = CODEC_TIMEOUT;
	do
	{
		word = readl( (&at91_ac97_private)->regs + AC97C_COSR );
		if( word & AT91C_AC97C_RXRDY )
		{
			ret = (u16) readl( (&at91_ac97_private)->regs + AC97C_CORHR );
			break;
		}

		udelay(1);
	} while (--timeout);

	if (!timeout)
		goto timed_out;

timed_out:
	up(&ac97_mutex);
	
	if( ret == 0xffff )
		snd_printk(KERN_WARNING "AT91 AC97 codec read timeout (got COSR: 0x%x)\n", word);
	
	DBG("%s: Register: 0x%x Value: 0x%x\n", __FUNCTION__, reg, ret);
	return ret;
}


/* Write data to the AC97 bus directly */
static void at91_ac97_write(struct snd_ac97 *ac97, unsigned short reg, unsigned short val)
{
	unsigned long word = 0x00;
	int timeout = CODEC_TIMEOUT;
	
	down(&ac97_mutex);
	
	DBG("%s: Writing codec register 0x%x = 0x%x\n", __FUNCTION__, reg, val);

	do
	{
		if ( readl( (&at91_ac97_private)->regs + AC97C_COSR ) & AT91C_AC97C_TXRDY )
		{
			word = (reg & 0x7f) << 16 | val;
			writel(word, (&at91_ac97_private)->regs + AC97C_COTHR);
			up(&ac97_mutex);
			return;
		}
		udelay(1);
	} while (--timeout);

	up(&ac97_mutex);
	snd_printk(KERN_WARNING "AT91 AC97 codec write timeout.\n");
}


/* Warm reset AC97. This is a soc thing... */
static void at91_ac97_warmrst(struct snd_ac97 *ac97)
{
	volatile unsigned int mr;
	
	snd_printk(KERN_INFO "AT91 AC97 Warm Reset\n");
	
	down(&ac97_mutex);
	
	mr = readl((&at91_ac97_private)->regs + AC97C_MR) | AT91C_AC97C_WRST;

	writel(mr, (&at91_ac97_private)->regs + AC97C_MR);
	udelay(1);

	mr &= ~AT91C_AC97C_WRST;
	writel(mr, (&at91_ac97_private)->regs + AC97C_MR);
	
	up(&ac97_mutex);
}


/* Cold reset AC97. This is a soc thing...
 * Enable AC97 Controller.
 * Perform a cold (hard) reset of the AC97 codec.
 */
static void at91_ac97_coldrst(struct snd_ac97 *ac97)
{
	snd_printk(KERN_INFO "AT91 AC97 Cold Reset\n");
	
	down(&ac97_mutex);
	
	// reset is active low
	at91_ac97_drive_reset(&at91_ac97_private, 0);
	
	// this delay *is* needed
	mdelay(50);
	
	at91_ac97_drive_reset(&at91_ac97_private, 1);
	
	// this delay is also needed
	mdelay(50);
	
	writel(0x00, (&at91_ac97_private)->regs + AC97C_MR);
	writel(AT91C_AC97C_ENA, (&at91_ac97_private)->regs + AC97C_MR);
	
	up(&ac97_mutex);
}


struct snd_ac97_bus_ops soc_ac97_ops = {
	.read		= at91_ac97_read,
	.write		= at91_ac97_write,
	.reset		= at91_ac97_coldrst,
	.warm_reset	= at91_ac97_warmrst,
};
EXPORT_SYMBOL_GPL(soc_ac97_ops);


/* When the AC97 controller interrupts it will enter this ISR */
static irqreturn_t at91_ac97_interrupt(int irq, void *dev_id)
{
	struct at91_ac97_info *ac97_info = dev_id;
	struct at91_pcm_dma_params *dma_params;
	u32 status;
	int i;

	// Read AC97 status register
	status = readl((&at91_ac97_private)->regs + AC97C_SR);

	// Got channel A event interrupt
	if (status & AT91C_AC97C_CAEVT) {
		// Get the channel A status register
		status = readl((&at91_ac97_private)->regs + AC97C_CASR);
		
		if (status & AT91C_AC97C_UNRUN) {
			snd_printk(KERN_WARNING "AT91 AC97 (CH-A) underrun,"
				   " status = 0x%16lx\n",
				   (long unsigned int)status);
		}
		
		if (status & AT91C_AC97C_OVRUN) {
			snd_printk(KERN_WARNING "AT91 AC97 (CH-A) overrun,"
				   " status = 0x%16lx\n",
				   (long unsigned int)status);
		}
		
		/*
		 * If a DMA related interrupt occured call the DMA interrupt
		 * handler to queue the next sample(s)
		 */
		for (i = 0; i < ARRAY_SIZE(ac97_info->dma_params); i++) {
			dma_params = ac97_info->dma_params[i];
			
			if (dma_params && dma_params->dma_enqueue && dma_params->substream) {
				if (status & (dma_params->dma_end | dma_params->dma_endbuf))
				{
					dma_params->dma_enqueue(status, dma_params->substream);
				}
			}
		}
	}

	return IRQ_HANDLED;
}


/*
 * Startup.  Only one substream allowed in each direction.
 */
static int at91_ac97_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	int dir_mask;
	
	DBG("%s: Entry\n", __FUNCTION__);
	
	spin_lock_irq(at91_ac97_private.lock);
	dir_mask = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)? 1:2;
	
	if (at91_ac97_private.dir_mask & dir_mask) {
		spin_unlock_irq(at91_ac97_private.lock);
		return -EBUSY;
	}
	
	at91_ac97_private.dir_mask |= dir_mask;
	spin_unlock_irq(at91_ac97_private.lock);
	
	return 0;
}


/*
 * Shutdown.  Clear DMA parameters and disable AC97 channel A if there
 * are no other substreams open.
 */
static void at91_ac97_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	int dir_mask;
	u32 word;

	DBG("%s: Entry\n", __FUNCTION__);
	
	dir_mask = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)? 1:2;
	at91_ac97_private.dir_mask &= ~dir_mask;
	
	if (!at91_ac97_private.dir_mask) {
		DBG("%s: no streams active\n", __FUNCTION__);
		
		// Disable channel A
		word = readl((&at91_ac97_private)->regs + AC97C_CAMR);
		word &= ~AT91C_AC97C_CEN;
		writel( word, (&at91_ac97_private)->regs + AC97C_CAMR );
	}
}


/*
 * Configure the AC97
 */
static int at91_ac97_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct at91_pcm_dma_params *dma_params;
	//int block_size = frames_to_bytes(runtime, runtime->period_size);
	int dir, channels, bits;
	u32 word;
	
	DBG("%s: Entry\n", __FUNCTION__);
	
	/*
	 * Currently, there is only one set of dma params for
	 * each direction.  If more are added, this code will
	 * have to be changed to select the proper set.
	 */
	dir = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)? 0:1;

	dma_params = &at91_ac97_dma_params[dir];
	dma_params->regs = (&at91_ac97_private)->regs;
	dma_params->substream = substream;

	at91_ac97_private.dma_params[dir] = dma_params;
	
	/*
	* The cpu_dai->dma_data field is only used to communicate the
	* appropriate DMA parameters to the pcm driver hw_params()
	* function.  It should not be used for other purposes
	* as it is common to all substreams.
	*/
	rtd->dai->cpu_dai->dma_data = dma_params;
	
	channels = params_channels(params);
	
	/*
	 * Compute AC97 register settings.
	 */
	
	/* Assign slots to channels */
	word = 0x00;
	switch (substream->runtime->channels) {
		case 1:
			word |= AT91C_AC97C_CHID3_CA;
			break;
		case 2:
		default:
			/* Assign Left and Right slots (3,4) to Channel A */
			word |= AT91C_AC97C_CHID3_CA | AT91C_AC97C_CHID4_CA;
			break;
	}
	
	writel( word, (&at91_ac97_private)->regs + AC97C_OCA );
	
	// Enable PDC
	word = AT91C_AC97C_PDCEN;
	
	// Enable channel A
	word |= AT91C_AC97C_CEN;
	
	/*
	 * Determine sample size in bits and the PDC increment.
	 * From datasheet:
	 *   The width of sample data, that transit via Channel A and Channel B varies and
	 *   can take one of these values; 10, 16, 18 or 20 bits.
	 * TODO support 10bit format
	 * FIXME: Avoid conflicts with capture channel.
	 */
	switch( params_format(params) ) {
		case SNDRV_PCM_FORMAT_S16_BE:
			word |= AT91C_AC97C_CEM;
		case SNDRV_PCM_FORMAT_S16_LE:
			word |= AT91C_AC97C_SIZE_16_BITS;
			bits = 16;
			dma_params->pdc_xfer_size = 2;
			break;
		
		case SNDRV_PCM_FORMAT_S18_3BE:
			word |= AT91C_AC97C_CEM;
		case SNDRV_PCM_FORMAT_S18_3LE:
			word |= AT91C_AC97C_SIZE_18_BITS;
			bits = 18;
			dma_params->pdc_xfer_size = 3;
			break;
		
		case SNDRV_PCM_FORMAT_S20_3BE:
			word |= AT91C_AC97C_CEM;
		case SNDRV_PCM_FORMAT_S20_3LE:
			word |= AT91C_AC97C_SIZE_20_BITS;
			bits = 20;
			dma_params->pdc_xfer_size = 3;
			break;
		
		default:
			snd_printk(KERN_WARNING "AT91 AC97: unsupported PCM format");
			return -EINVAL;
	}
	
	
	// Write the Channel A mode register
	writel( word, (&at91_ac97_private)->regs + AC97C_CAMR );
	
	/* Set variable rate if needed */
	word = readl( (&at91_ac97_private)->regs + AC97C_MR );
	if ( runtime->rate != 48000 ){
		/* Set Variable Rate Bit */
		word |= AT91C_AC97C_VRA;
		DBG("%s: Enable variable rate %d\n", __FUNCTION__, runtime->rate);
	} else{
		/* Clear Variable Rate Bit */
		word &= ~AT91C_AC97C_VRA;
	}
	
	writel( word, (&at91_ac97_private)->regs + AC97C_MR );
	
	DBG("%s: AC97 Initialised\n", __FUNCTION__);
	
	return 0;
}


/* Triggered by pcm playback start/stop/pause etc. */
static int at91_ac97_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	u32 word;
	
	DBG("%s: Entry\n", __FUNCTION__);
	
	word = readl((&at91_ac97_private)->regs + AC97C_CAMR);
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		// Enable Interrupts (for PDC transfers)
		switch (substream->stream) {
		case SNDRV_PCM_STREAM_PLAYBACK: /* TX stream */
			word |= (AT91C_AC97C_ENDTX | AT91C_AC97C_TXBUFE);
			break;
		
		case SNDRV_PCM_STREAM_CAPTURE: /* RX stream */
			word |= (AT91C_AC97C_ENDRX | AT91C_AC97C_RXBUFF);
			break;
		}
		break;
		
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		// Disable Interrupts (for PDC transfers)
		switch (substream->stream) {
		case SNDRV_PCM_STREAM_PLAYBACK: /* TX stream */
			word &= ~(AT91C_AC97C_ENDTX | AT91C_AC97C_TXBUFE);
			break;
		
		case SNDRV_PCM_STREAM_CAPTURE: /* RX stream */
			word &= ~(AT91C_AC97C_ENDRX | AT91C_AC97C_RXBUFF);
			break;
		}
		break;
		
	default:
		return -EINVAL;
	}
	
	// Write the Channel A mode register
	writel( word, (&at91_ac97_private)->regs + AC97C_CAMR );
	
	return 0;
}


#ifdef CONFIG_PM
static int at91_ac97_suspend(struct snd_soc_dai *cpu_dai)
{
	DBG("%s: Entry\n", __FUNCTION__);
	return 0;
}

static int at91_ac97_resume(struct snd_soc_dai *cpu_dai)
{
	DBG("%s: Entry\n", __FUNCTION__);
	return 0;
}
#else
#define at91_ac97_suspend	NULL
#define at91_ac97_resume	NULL
#endif


static int at91_ac97_soc_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	u32 word;
	
	snd_printk(KERN_INFO "at91_ac97 initialise codec controller.\n");
	
	// Pull nReset line high
	at91_ac97_drive_reset(&at91_ac97_private, 1);
	
	// Delay until codec crystal starts
	mdelay(50);
	
	// Enable AC97 controller
	writel(AT91C_AC97C_ENA, (&at91_ac97_private)->regs + AC97C_MR );
	
	// Enable codec events interrupts
	word = AT91C_AC97C_TXRDY | AT91C_AC97C_RXRDY;
	writel(word, (&at91_ac97_private)->regs + AC97C_COMR );
	
	/* Enable Channel A interrupts */
	/* Enable, Unmask interrups */
	word = AT91C_AC97C_CAEVT;
	writel(word, (&at91_ac97_private)->regs + AC97C_IER );
	writel(word, (&at91_ac97_private)->regs + AC97C_IMR );
	
	/* Reset PDC registers */
	writel(0, (&at91_ac97_private)->regs + ATMEL_PDC_RPR);
	writel(0, (&at91_ac97_private)->regs + ATMEL_PDC_RCR);
	writel(0, (&at91_ac97_private)->regs + ATMEL_PDC_RNPR);
	writel(0, (&at91_ac97_private)->regs + ATMEL_PDC_RNCR);
	writel(0, (&at91_ac97_private)->regs + ATMEL_PDC_TPR);
	writel(0, (&at91_ac97_private)->regs + ATMEL_PDC_TCR);
	writel(0, (&at91_ac97_private)->regs + ATMEL_PDC_TNPR);
	writel(0, (&at91_ac97_private)->regs + ATMEL_PDC_TNCR);
	
	/* No streams active */
	at91_ac97_private.dir_mask = 0x00;
	
	return 0;
}


static void at91_ac97_soc_remove(struct platform_device *pdev, struct snd_soc_dai *cpu_dai)
{
	snd_printk(KERN_INFO "at91_ac97 shutdown codec controller\n" );
	
	// Disable AC97 Controller
	writel(0, (&at91_ac97_private)->regs + AC97C_MR);

	// Pull nReset low
	at91_ac97_drive_reset(&at91_ac97_private, 0);
}


#define AT91_AC97_RATES SNDRV_PCM_RATE_8000_48000
#define AT91_AC97_FMTS (SNDRV_PCM_FMTBIT_S16_LE|SNDRV_PCM_FORMAT_S16_BE|\
			SNDRV_PCM_FORMAT_S18_3LE|SNDRV_PCM_FORMAT_S18_3BE|\
			SNDRV_PCM_FORMAT_S20_3LE|SNDRV_PCM_FORMAT_S20_3BE)


static struct snd_soc_dai_ops at91_ac97_dai_ops = {
	.startup	= at91_ac97_startup,
	.shutdown	= at91_ac97_shutdown,
	.trigger	= at91_ac97_trigger,
	.hw_params	= at91_ac97_hw_params,

};


struct snd_soc_dai at91_ac97_dai[] = {
{
	.name			= "at91-ac97",
	.id			= 0,
	.ac97_control		= 1,
	.suspend		= at91_ac97_suspend,
	.resume			= at91_ac97_resume,
	.probe			= at91_ac97_soc_probe,
	.remove			= at91_ac97_soc_remove,
	.playback = {
		.stream_name = "AC97 Playback",
		.rates		= AT91_AC97_RATES,
		.formats	= AT91_AC97_FMTS,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.capture = {
		.stream_name = "AC97 Capture",
		.rates		= AT91_AC97_RATES,
		.formats	= AT91_AC97_FMTS,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.ops = &at91_ac97_dai_ops,
	.private_data = &at91_ac97_private,
},
};
EXPORT_SYMBOL_GPL(at91_ac97_dai);


static int at91_ac97_probe(struct platform_device *pdev)
{
	struct atmel_ac97_data *pdata = pdev->dev.platform_data;
	int err = 0;
	
	// Get the nReset pin
	at91_ac97_private.reset_pin = pdata->reset_pin;

	if (!(pdev->resource[0].flags & IORESOURCE_MEM) || !(pdev->resource[1].flags & IORESOURCE_IRQ))
	{
		printk(KERN_WARNING "at91_ac97 probe failed: incorrect resource\n");
		return -ENODEV;
	}
	
	// map the registers
	at91_ac97_private.regs = ioremap(pdev->resource[0].start, pdev->resource[0].end - pdev->resource[0].start + 1);
	if (at91_ac97_private.regs == NULL) {
		snd_printk(KERN_ERR "at91_ac97 probe failed: Unable to remap IO memory.\n");
	
		err = -ENXIO;
		goto err_all;
	}
	
	// Request IRQ
	at91_ac97_private.irq = pdev->resource[1].start;
	err = request_irq(at91_ac97_private.irq, at91_ac97_interrupt, IRQF_DISABLED, "ac97", &at91_ac97_private);
	if (err < 0) {
		snd_printk(KERN_ERR "at91_ac97 probe failed: Unable to request IRQ-%d\n", at91_ac97_private.irq);
		
		err = -ENXIO;
		goto err_regs;
	}
	printk(KERN_INFO "at91_ac97 probe IRQ=%d\n", at91_ac97_private.irq);
	
	// Enable AC97 Controller clock
	at91_ac97_private.ac97_clk = clk_get(NULL, "ac97_clk");
	if(at91_ac97_private.ac97_clk == NULL) {
		snd_printk(KERN_ERR "AT91 AC97 Failed to get clock.\n");
		err = -ENODEV;
		goto err_irq;
	}
	clk_enable(at91_ac97_private.ac97_clk);

	err = snd_soc_register_dais(at91_ac97_dai, ARRAY_SIZE(at91_ac97_dai));
	if(err)
	{
		snd_printk(KERN_ERR "at91_ac97 probe failed: Unable to register DAI\n");
		err = -ENODEV;
		goto err_clk;
	}

	return 0;

err_clk:
	clk_disable(at91_ac97_private.ac97_clk);
	clk_put(at91_ac97_private.ac97_clk);

err_irq:
	free_irq(at91_ac97_private.irq, &at91_ac97_private);

err_regs:
	iounmap(at91_ac97_private.regs);

err_all:	
	return err;
}


static int at91_ac97_remove(struct platform_device *pdev)
{
	// stop AC97 clock
	clk_disable(at91_ac97_private.ac97_clk);
	clk_put(at91_ac97_private.ac97_clk);

	// unmap registers
	iounmap(at91_ac97_private.regs);

	// free IRQ
	free_irq(at91_ac97_private.irq, &at91_ac97_private);

	snd_soc_unregister_dais(at91_ac97_dai, ARRAY_SIZE(at91_ac97_dai));

	return 0;
}

static struct platform_driver at91_ac97_driver =
{
	.probe      = at91_ac97_probe,
	.remove     = at91_ac97_remove,
	.driver     = {
		.name       = "at91-ac97",
	},
};


static int __init at91_ac97_init(void)
{
	return platform_driver_register(&at91_ac97_driver);
}


static void __exit at91_ac97_exit(void)
{
	platform_driver_unregister(&at91_ac97_driver);
}


module_init(at91_ac97_init);
module_exit(at91_ac97_exit);

/* Module information */
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("alsa AC97 driver for AT91 SoC");
MODULE_AUTHOR("Matthew Dombroski <mdombroski@4d-electronics.co.nz>");
