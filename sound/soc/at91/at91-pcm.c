/*
 * sound/soc/at91/at91-pcm.c --  ALSA Soc Audio Layer
 * PCM part - dma, dai
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
 *    04 March 2009	Port to linux-2.6.28
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/atmel_pdc.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/hardware.h>

#include "at91-pcm.h"

#define AT91_PCM_DEBUG 0
#if AT91_PCM_DEBUG
#  define DBG(x...) printk(KERN_WARNING "at91-pcm: " x)
#else
#  define DBG(x...)
#endif

#define PCM_FMTS SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S8

static struct snd_pcm_hardware at91_pcm_hw = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER |
				  SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= PCM_FMTS,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= 64*1024, /* 16 bit dma counter */
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= PAGE_SIZE*2,
	.periods_min		= 2,
	.periods_max		= 128,
	.fifo_size		= 32,
};


struct at91_pcm_runtime_data {
	struct at91_pcm_dma_params *params;
	dma_addr_t dma_buffer;			/* physical address of dma buffer */
	dma_addr_t dma_buffer_end;		/* first address beyond DMA buffer */
	size_t period_size;
	dma_addr_t period_ptr;			/* physical address of next period */
	u32 pdc_xpr_save;			/* PDC register save */
	u32 pdc_xcr_save;
	u32 pdc_xnpr_save;
	u32 pdc_xncr_save;
	spinlock_t lock;
};



/*
 * This function updates the PDC (DMA) parameters.
 * It is run in an interrupt context
 */
static void at91_pcm_dma_enqueue(u32 sr, struct snd_pcm_substream *substream)
{
	struct at91_pcm_runtime_data *prtd;
	struct at91_pcm_dma_params *params;
	
	prtd = substream->runtime->private_data;
	params = prtd->params;
	
	// dma buffer ended unexpectedly - underrrun or overrun
	if (sr & params->dma_endbuf) {
		printk(KERN_WARNING "at91-pcm: buffer %s on %s (SR=%#x)\n",
		       substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "underrun" : "overrun",
		       params->name, sr);

		/* re-start the PDC */
		writel(params->dma_disable, params->regs + params->pdc->ptcr);

		prtd->period_ptr += prtd->period_size;
		if (prtd->period_ptr >= prtd->dma_buffer_end) {
			prtd->period_ptr = prtd->dma_buffer;
		}

		writel(prtd->period_ptr, params->regs + params->pdc->xpr);
		writel(prtd->period_size / params->pdc_xfer_size, params->regs + params->pdc->xcr);

		writel(params->dma_enable, params->regs + params->pdc->ptcr);
	}

	// dma transfer ended, set up next transfer
	if (sr & params->dma_end) {
		/* Load the PDC next pointer and counter registers */
		prtd->period_ptr += prtd->period_size;
		
		// If we overflowed the buffer then restart transfer
		if (prtd->period_ptr >= prtd->dma_buffer_end) {
			prtd->period_ptr = prtd->dma_buffer;
		}
		
		// Configure PDC for next transfer
		writel(prtd->period_ptr, params->regs + params->pdc->xnpr);
		writel(prtd->period_size / params->pdc_xfer_size, params->regs + params->pdc->xncr);
	}

	snd_pcm_period_elapsed(substream);
}



/*
 * This is called when the hardware parameter (hw_params) is set up by the application, that is,
 * once when the buffer size, the period size, the format, etc. are defined for the pcm substream.
 *
 * Many hardware set-up tasks should be done in this callback, including the allocation of buffers.
 *
 * this may get called several times by oss emulation
 * with different params
 */
static int at91_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct at91_pcm_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	DBG("%s: Entry\n", __FUNCTION__);
	
	spin_lock_irq(&prtd->lock);
	
	// Set DMA
	prtd->params = rtd->dai->cpu_dai->dma_data;
	prtd->params->dma_enqueue = at91_pcm_dma_enqueue;
	
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->dma_buffer = runtime->dma_addr;
	prtd->dma_buffer_end = runtime->dma_addr + runtime->dma_bytes;
	prtd->period_size = params_period_bytes(params);

	DBG("%s: DMA for %s configured (dma_bytes=%d, period_size=%d)\n",
	    __FUNCTION__, prtd->params->name,
	    runtime->dma_bytes, prtd->period_size);
	
	spin_unlock_irq(&prtd->lock);
	return 0;
}


/*
 * This is called to release the resources allocated via hw_params.
 */
static int at91_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct at91_pcm_runtime_data *prtd = substream->runtime->private_data;
	struct at91_pcm_dma_params *params = prtd->params;
	
	DBG("%s: Entry\n", __FUNCTION__);
	
	if (params != NULL) {
		writel(params->dma_disable, params->regs + ATMEL_PDC_PTCR);
		prtd->params->dma_enqueue = 0x00;
	}
	
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}


/*
 * This is called when the pcm playback is started, stopped or paused.
 *
 * Which action is specified in the second argument, SNDRV_PCM_TRIGGER_XXX in <sound/pcm.h>.
 * At least, START and STOP commands must be defined in this callback.
 */
static int at91_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct at91_pcm_runtime_data *prtd = substream->runtime->private_data;
	struct at91_pcm_dma_params *params = prtd->params;
	int ret = 0;

	DBG("%s: Entry\n", __FUNCTION__);
	spin_lock(&prtd->lock);
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		// set up first transfer
		prtd->period_ptr = prtd->dma_buffer;
		writel(prtd->period_ptr, params->regs + params->pdc->xpr);
		writel(prtd->period_size / params->pdc_xfer_size, params->regs + params->pdc->xcr);

		// set up next transfer
		prtd->period_ptr += prtd->period_size;
		writel(prtd->period_ptr, params->regs + params->pdc->xnpr);
		writel(prtd->period_size / params->pdc_xfer_size, params->regs + params->pdc->xncr);
		
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		DBG("%s: Enable PDC\n", __FUNCTION__);
		
		/* Start DMA */
		writel(params->dma_enable, params->regs + params->pdc->ptcr);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		DBG("%s: Disable PDC\n", __FUNCTION__);
		
		/* Stop DMA */
		writel(params->dma_disable, params->regs + params->pdc->ptcr);
		break;

	default:
		ret = -EINVAL;
	}
	
	spin_unlock(&prtd->lock);
	
	return ret;
}


static  snd_pcm_uframes_t at91_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct at91_pcm_runtime_data *prtd = runtime->private_data;
	struct at91_pcm_dma_params *params = prtd->params;
	snd_pcm_uframes_t pos;
	dma_addr_t ptr;

	spin_lock(&prtd->lock);
	
	// Use DMA to get the bytes transferred and the pointer.
	ptr = (dma_addr_t) readl(params->regs + params->pdc->xpr);
	pos = bytes_to_frames(runtime, ptr - prtd->dma_buffer);

	if (pos == runtime->buffer_size)
		pos = 0;
	
	spin_unlock(&prtd->lock);
	
	return pos;
}


/*
 * This is called when a pcm substream is opened.
 */
static int at91_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct at91_pcm_runtime_data *prtd;
	
	DBG("%s: Entry\n", __FUNCTION__);

	snd_soc_set_runtime_hwparams(substream, &at91_pcm_hw);

	/* ensure that buffer size is a multiple of period size */
	if (snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS) < 0)
		return -EINVAL;

	prtd = kzalloc(sizeof(struct at91_pcm_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;
	
	spin_lock_init(&prtd->lock);
	
	runtime->private_data = prtd;
	return 0;
}



/*
 * this is called when a pcm substream is closed.
 */
static int at91_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct at91_pcm_runtime_data *prtd = runtime->private_data;
	
	DBG("%s: Entry\n", __FUNCTION__);

	if(prtd)
		kfree(prtd);
	else
		DBG("at91_pcm_close called with prtd == NULL\n");
	
	return 0;
}


static int at91_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;

	DBG("%s: Entry\n", __FUNCTION__);

	ret = dma_mmap_writecombine(substream->pcm->card->dev, vma,
				    runtime->dma_area,
				    runtime->dma_addr,
				    runtime->dma_bytes);
	
	return ret;
}


static struct snd_pcm_ops at91_pcm_ops = {
	.open		= at91_pcm_open,
	.close		= at91_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= at91_pcm_hw_params,
	.hw_free	= at91_pcm_hw_free,
	.trigger	= at91_pcm_trigger,
	.pointer	= at91_pcm_pointer,
	.mmap		= at91_pcm_mmap,
};


static int at91_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = at91_pcm_hw.buffer_bytes_max;

	DBG("%s: Entry\n", __FUNCTION__);

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	
	DBG("preallocate_dma_buffer: area=%p, addr=%p, size=%d\n",
	    (void *) buf->area,
	    (void *) buf->addr,
	     size);
	
	if (!buf->area)
		return -ENOMEM;
	
	buf->bytes = size;
	return 0;
}


static void at91_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	DBG("%s: Entry\n", __FUNCTION__);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes, buf->area, buf->addr);
		buf->area = NULL;
	}
}


static u64 at91_pcm_dmamask = DMA_32BIT_MASK;


/* checked */
static int at91_pcm_new(struct snd_card *card, struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int err = 0;

	DBG("%s: Entry\n", __FUNCTION__);
	
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &at91_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

	if (dai->playback.channels_min) {
		err = at91_pcm_preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
		if (err)
			goto out;
	}

	if (dai->capture.channels_min) {
		err = at91_pcm_preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_CAPTURE);
		if (err)
			goto out;
	}
	
out:
	return err;
}


struct snd_soc_platform at91_soc_platform = {
	.name		= "at91-pcm",
	.pcm_ops 	= &at91_pcm_ops,
	.pcm_new	= at91_pcm_new,
	.pcm_free	= at91_pcm_free_dma_buffers,
};

EXPORT_SYMBOL_GPL(at91_soc_platform);


static int __init at91_pcm_platform_init(void)
{
	return snd_soc_register_platform(&at91_soc_platform);
}

static void __exit at91_pcm_platform_exit(void)
{
	snd_soc_unregister_platform(&at91_soc_platform);
}


module_init(at91_pcm_platform_init);
module_exit(at91_pcm_platform_exit);


MODULE_AUTHOR("Matthew Dombroski, <mdombroski@4d-electronics.co.nz>");
MODULE_DESCRIPTION("Atmel AT91 PCM interface");
MODULE_LICENSE("GPL");
