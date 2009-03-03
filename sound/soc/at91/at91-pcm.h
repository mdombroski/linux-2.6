/*
 * sound/soc/at91/at91-pcm.h --  ALSA Soc Audio Layer
 *
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
 

#ifndef _SND_SOC_AT91_PCM_H
#define _SND_SOC_AT91_PCM_H

/*
 * PDC registers needed for DMA control.
 */
struct at91_pdc_regs {
	u32 xpr;	/* Pointer register */
	u32 xcr;	/* Counter register */
	u32 xnpr;	/* Next pointer register */
	u32 xncr;	/* Next counter register */
	u32 ptcr;	/* Transfer control register */
};

/*
 * Atmel AT91 has a Peripheral Data Controller for the AC97 and SSC
 * peripherals. pdc_regs points to the registers for the peripheral, then the
 * offsets in linux/atmel_pdc.h are used to read/write dma information.
 */
struct at91_pcm_dma_params {
	char* name;		/* Stream identifier */
	void __iomem *regs;	/* PDC Registers */
	int pdc_xfer_size;	/* PDC counter increment in bytes */
	/* pcm substream */
	struct snd_pcm_substream *substream;
	
	/* Registers for PDC */
	struct at91_pdc_regs *pdc; /* PDC receive or transmit registers */
	
	/* Flags for PDC (DMA) */
	u32 dma_end;		/* ENDTX or ENDRX */
	u32 dma_endbuf;		/* TXEMPTY or */
	u32 dma_enable;		/* PDC enable flag (TXTEN or RXTEN) */
	u32 dma_disable;	/* PDC disable flag (TXTDIS or RXTDIS) */
	
	/* Enqueue next DMA block */
	void (*dma_enqueue)(u32 sr, struct snd_pcm_substream *);
};


/* platform data */
extern struct snd_soc_platform at91_soc_platform;


#endif /* _SND_SOC_AT91_PCM_H */
