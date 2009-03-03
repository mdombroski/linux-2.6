/*
 * linux/include/asm-arm/arch-at91/at91_ac97c.h
 *
 * Hardware definition for the ac97c peripheral in the ATMEL at91sam926x processor
 *
 * Generated  12/09/2005 (11:54:20) AT91 SW Application Group from AC97C_XXXX V1.3
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __AT91_AC97C_H
#define __AT91_AC97C_H

/* -------------------------------------------------------- */
/* AC97C ID definitions for  AT91SAM926x           */
/* -------------------------------------------------------- */
#ifndef AT91C_ID_AC97C
#define AT91C_ID_AC97C 	18 /**< AC97 Controller id */
#endif /* AT91C_ID_AC97C */

/* -------------------------------------------------------- */
/* AC97C Base Address definitions for  AT91SAM926x   */
/* -------------------------------------------------------- */
#define AT91C_BASE_AC97C     	0xFFFA0000 /**< AC97C base address */

/* -------------------------------------------------------- */
/* PIO definition for AC97C hardware peripheral */
/* -------------------------------------------------------- */
#define AT91C_PB1_AC97CK   	(1 << 1) /**<  */
#define AT91C_PB0_AC97FS   	(1 << 0) /**<  */
#define AT91C_PB3_AC97RX   	(1 << 3) /**<  */
#define AT91C_PB2_AC97TX   	(1 << 2) /**<  */


/* -------------------------------------------------------- */
/* Register offset definition for AC97C hardware peripheral */
/* -------------------------------------------------------- */
#define AC97C_MR 	(0x0008) 	/**< Mode Register */
#define AC97C_ICA 	(0x0010) 	/**< Input Channel AssignementRegister */
#define AC97C_OCA 	(0x0014) 	/**< Output Channel Assignement Register */
#define AC97C_CARHR 	(0x0020) 	/**< Channel A Receive Holding Register */
#define AC97C_CATHR 	(0x0024) 	/**< Channel A Transmit Holding Register */
#define AC97C_CASR 	(0x0028) 	/**< Channel A Status Register */
#define AC97C_CAMR 	(0x002C) 	/**< Channel A Mode Register */
#define AC97C_CBRHR 	(0x0030) 	/**< Channel B Receive Holding Register (optional) */
#define AC97C_CBTHR 	(0x0034) 	/**< Channel B Transmit Holding Register (optional) */
#define AC97C_CBSR 	(0x0038) 	/**< Channel B Status Register */
#define AC97C_CBMR 	(0x003C) 	/**< Channel B Mode Register */
#define AC97C_CORHR 	(0x0040) 	/**< COdec Transmit Holding Register */
#define AC97C_COTHR 	(0x0044) 	/**< COdec Transmit Holding Register */
#define AC97C_COSR 	(0x0048) 	/**< CODEC Status Register */
#define AC97C_COMR 	(0x004C) 	/**< CODEC Mask Status Register */
#define AC97C_SR 	(0x0050) 	/**< Status Register */
#define AC97C_IER 	(0x0054) 	/**< Interrupt Enable Register */
#define AC97C_IDR 	(0x0058) 	/**< Interrupt Disable Register */
#define AC97C_IMR 	(0x005C) 	/**< Interrupt Mask Register */
#define AC97C_VERSION 	(0x00FC) 	/**< Version Register */

/* -------------------------------------------------------- */
/* Bitfields definition for AC97C hardware peripheral */
/* -------------------------------------------------------- */
/* --- Register AC97C_MR */
#define AT91C_AC97C_ENA       (0x1 << 0 ) /**< (AC97C) AC97 Controller Global Enable */
#define AT91C_AC97C_WRST      (0x1 << 1 ) /**< (AC97C) Warm Reset */
#define AT91C_AC97C_VRA       (0x1 << 2 ) /**< (AC97C) Variable RAte (for Data Slots) */
/* --- Register AC97C_ICA */
#define AT91C_AC97C_CHID3     (0x7 << 0 ) /**< (AC97C) Channel Id for the input slot 3 */
#define 	AT91C_AC97C_CHID3_NONE                 0x0 /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID3_CA                   0x1 /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID3_CB                   0x2 /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID3_CC                   0x3 /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID4     (0x7 << 3 ) /**< (AC97C) Channel Id for the input slot 4 */
#define 	AT91C_AC97C_CHID4_NONE                 (0x0 <<  3) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID4_CA                   (0x1 <<  3) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID4_CB                   (0x2 <<  3) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID4_CC                   (0x3 <<  3) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID5     (0x7 << 6 ) /**< (AC97C) Channel Id for the input slot 5 */
#define 	AT91C_AC97C_CHID5_NONE                 (0x0 <<  6) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID5_CA                   (0x1 <<  6) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID5_CB                   (0x2 <<  6) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID5_CC                   (0x3 <<  6) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID6     (0x7 << 9 ) /**< (AC97C) Channel Id for the input slot 6 */
#define 	AT91C_AC97C_CHID6_NONE                 (0x0 <<  9) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID6_CA                   (0x1 <<  9) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID6_CB                   (0x2 <<  9) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID6_CC                   (0x3 <<  9) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID7     (0x7 << 12) /**< (AC97C) Channel Id for the input slot 7 */
#define 	AT91C_AC97C_CHID7_NONE                 (0x0 << 12) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID7_CA                   (0x1 << 12) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID7_CB                   (0x2 << 12) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID7_CC                   (0x3 << 12) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID8     (0x7 << 15) /**< (AC97C) Channel Id for the input slot 8 */
#define 	AT91C_AC97C_CHID8_NONE                 (0x0 << 15) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID8_CA                   (0x1 << 15) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID8_CB                   (0x2 << 15) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID8_CC                   (0x3 << 15) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID9     (0x7 << 18) /**< (AC97C) Channel Id for the input slot 9 */
#define 	AT91C_AC97C_CHID9_NONE                 (0x0 << 18) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID9_CA                   (0x1 << 18) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID9_CB                   (0x2 << 18) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID9_CC                   (0x3 << 18) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID10    (0x7 << 21) /**< (AC97C) Channel Id for the input slot 10 */
#define 	AT91C_AC97C_CHID10_NONE                 (0x0 << 21) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID10_CA                   (0x1 << 21) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID10_CB                   (0x2 << 21) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID10_CC                   (0x3 << 21) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID11    (0x7 << 24) /**< (AC97C) Channel Id for the input slot 11 */
#define 	AT91C_AC97C_CHID11_NONE                 (0x0 << 24) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID11_CA                   (0x1 << 24) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID11_CB                   (0x2 << 24) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID11_CC                   (0x3 << 24) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID12    (0x7 << 27) /**< (AC97C) Channel Id for the input slot 12 */
#define 	AT91C_AC97C_CHID12_NONE                 (0x0 << 27) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID12_CA                   (0x1 << 27) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID12_CB                   (0x2 << 27) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID12_CC                   (0x3 << 27) /**< (AC97C) Channel C data will be transmitted during this slot */
/* --- Register AC97C_OCA */
#define AT91C_AC97C_CHID3     (0x7 << 0 ) /**< (AC97C) Channel Id for the input slot 3 */
#define 	AT91C_AC97C_CHID3_NONE                 0x0 /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID3_CA                   0x1 /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID3_CB                   0x2 /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID3_CC                   0x3 /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID4     (0x7 << 3 ) /**< (AC97C) Channel Id for the input slot 4 */
#define 	AT91C_AC97C_CHID4_NONE                 (0x0 <<  3) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID4_CA                   (0x1 <<  3) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID4_CB                   (0x2 <<  3) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID4_CC                   (0x3 <<  3) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID5     (0x7 << 6 ) /**< (AC97C) Channel Id for the input slot 5 */
#define 	AT91C_AC97C_CHID5_NONE                 (0x0 <<  6) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID5_CA                   (0x1 <<  6) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID5_CB                   (0x2 <<  6) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID5_CC                   (0x3 <<  6) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID6     (0x7 << 9 ) /**< (AC97C) Channel Id for the input slot 6 */
#define 	AT91C_AC97C_CHID6_NONE                 (0x0 <<  9) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID6_CA                   (0x1 <<  9) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID6_CB                   (0x2 <<  9) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID6_CC                   (0x3 <<  9) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID7     (0x7 << 12) /**< (AC97C) Channel Id for the input slot 7 */
#define 	AT91C_AC97C_CHID7_NONE                 (0x0 << 12) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID7_CA                   (0x1 << 12) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID7_CB                   (0x2 << 12) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID7_CC                   (0x3 << 12) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID8     (0x7 << 15) /**< (AC97C) Channel Id for the input slot 8 */
#define 	AT91C_AC97C_CHID8_NONE                 (0x0 << 15) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID8_CA                   (0x1 << 15) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID8_CB                   (0x2 << 15) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID8_CC                   (0x3 << 15) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID9     (0x7 << 18) /**< (AC97C) Channel Id for the input slot 9 */
#define 	AT91C_AC97C_CHID9_NONE                 (0x0 << 18) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID9_CA                   (0x1 << 18) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID9_CB                   (0x2 << 18) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID9_CC                   (0x3 << 18) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID10    (0x7 << 21) /**< (AC97C) Channel Id for the input slot 10 */
#define 	AT91C_AC97C_CHID10_NONE                 (0x0 << 21) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID10_CA                   (0x1 << 21) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID10_CB                   (0x2 << 21) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID10_CC                   (0x3 << 21) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID11    (0x7 << 24) /**< (AC97C) Channel Id for the input slot 11 */
#define 	AT91C_AC97C_CHID11_NONE                 (0x0 << 24) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID11_CA                   (0x1 << 24) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID11_CB                   (0x2 << 24) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID11_CC                   (0x3 << 24) /**< (AC97C) Channel C data will be transmitted during this slot */
#define AT91C_AC97C_CHID12    (0x7 << 27) /**< (AC97C) Channel Id for the input slot 12 */
#define 	AT91C_AC97C_CHID12_NONE                 (0x0 << 27) /**< (AC97C) No data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID12_CA                   (0x1 << 27) /**< (AC97C) Channel A data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID12_CB                   (0x2 << 27) /**< (AC97C) Channel B data will be transmitted during this slot */
#define 	AT91C_AC97C_CHID12_CC                   (0x3 << 27) /**< (AC97C) Channel C data will be transmitted during this slot */
/* --- Register AC97C_CARHR */
#define AT91C_AC97C_RDATA     (0xFFFFF << 0 ) /**< (AC97C) Receive data */
/* --- Register AC97C_CATHR */
#define AT91C_AC97C_TDATA     (0xFFFFF << 0 ) /**< (AC97C) Transmit data */
/* --- Register AC97C_CASR */
#define AT91C_AC97C_TXRDY     (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_TXEMPTY   (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_UNRUN     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_RXRDY     (0x1 << 4 ) /**< (AC97C)  */
#define AT91C_AC97C_OVRUN     (0x1 << 5 ) /**< (AC97C)  */
#define AT91C_AC97C_ENDTX     (0x1 << 10) /**< (AC97C)  */
#define AT91C_AC97C_TXBUFE    (0x1 << 11) /**< (AC97C)  */
#define AT91C_AC97C_ENDRX     (0x1 << 14) /**< (AC97C)  */
#define AT91C_AC97C_RXBUFF    (0x1 << 15) /**< (AC97C)  */
/* --- Register AC97C_CAMR */
#define AT91C_AC97C_TXRDY     (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_TXEMPTY   (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_UNRUN     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_RXRDY     (0x1 << 4 ) /**< (AC97C)  */
#define AT91C_AC97C_OVRUN     (0x1 << 5 ) /**< (AC97C)  */
#define AT91C_AC97C_ENDTX     (0x1 << 10) /**< (AC97C)  */
#define AT91C_AC97C_TXBUFE    (0x1 << 11) /**< (AC97C)  */
#define AT91C_AC97C_ENDRX     (0x1 << 14) /**< (AC97C)  */
#define AT91C_AC97C_RXBUFF    (0x1 << 15) /**< (AC97C)  */
#define AT91C_AC97C_SIZE      (0x3 << 16) /**< (AC97C)  */
#define 	AT91C_AC97C_SIZE_20_BITS              (0x0 << 16) /**< (AC97C) Data size is 20 bits */
#define 	AT91C_AC97C_SIZE_18_BITS              (0x1 << 16) /**< (AC97C) Data size is 18 bits */
#define 	AT91C_AC97C_SIZE_16_BITS              (0x2 << 16) /**< (AC97C) Data size is 16 bits */
#define 	AT91C_AC97C_SIZE_10_BITS              (0x3 << 16) /**< (AC97C) Data size is 10 bits */
#define AT91C_AC97C_CEM       (0x1 << 18) /**< (AC97C)  */
#define AT91C_AC97C_CEN       (0x1 << 21) /**< (AC97C)  */
#define AT91C_AC97C_PDCEN     (0x1 << 22) /**< (AC97C)  */
/* --- Register AC97C_CBRHR */
#define AT91C_AC97C_RDATA     (0xFFFFF << 0 ) /**< (AC97C) Receive data */
/* --- Register AC97C_CBTHR */
#define AT91C_AC97C_TDATA     (0xFFFFF << 0 ) /**< (AC97C) Transmit data */
/* --- Register AC97C_CBSR */
#define AT91C_AC97C_TXRDY     (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_TXEMPTY   (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_UNRUN     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_RXRDY     (0x1 << 4 ) /**< (AC97C)  */
#define AT91C_AC97C_OVRUN     (0x1 << 5 ) /**< (AC97C)  */
/* --- Register AC97C_CBMR */
#define AT91C_AC97C_TXRDY     (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_TXEMPTY   (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_UNRUN     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_RXRDY     (0x1 << 4 ) /**< (AC97C)  */
#define AT91C_AC97C_OVRUN     (0x1 << 5 ) /**< (AC97C)  */
#define AT91C_AC97C_SIZE      (0x3 << 16) /**< (AC97C)  */
#define 	AT91C_AC97C_SIZE_20_BITS              (0x0 << 16) /**< (AC97C) Data size is 20 bits */
#define 	AT91C_AC97C_SIZE_18_BITS              (0x1 << 16) /**< (AC97C) Data size is 18 bits */
#define 	AT91C_AC97C_SIZE_16_BITS              (0x2 << 16) /**< (AC97C) Data size is 16 bits */
#define 	AT91C_AC97C_SIZE_10_BITS              (0x3 << 16) /**< (AC97C) Data size is 10 bits */
#define AT91C_AC97C_CEM       (0x1 << 18) /**< (AC97C)  */
#define AT91C_AC97C_CEN       (0x1 << 21) /**< (AC97C)  */
/* --- Register AC97C_CORHR */
#define AT91C_AC97C_SDATA     (0xFFFF << 0 ) /**< (AC97C) Status Data */
/* --- Register AC97C_COTHR */
#define AT91C_AC97C_CDATA     (0xFFFF << 0 ) /**< (AC97C) Command Data */
#define AT91C_AC97C_CADDR     (0x7F << 16) /**< (AC97C) COdec control register index */
#define AT91C_AC97C_READ      (0x1 << 23) /**< (AC97C) Read/Write command */
/* --- Register AC97C_COSR */
#define AT91C_AC97C_TXRDY     (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_TXEMPTY   (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_UNRUN     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_RXRDY     (0x1 << 4 ) /**< (AC97C)  */
/* --- Register AC97C_COMR */
#define AT91C_AC97C_TXRDY     (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_TXEMPTY   (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_UNRUN     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_RXRDY     (0x1 << 4 ) /**< (AC97C)  */
/* --- Register AC97C_SR */
#define AT91C_AC97C_SOF       (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_WKUP      (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_COEVT     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_CAEVT     (0x1 << 3 ) /**< (AC97C)  */
#define AT91C_AC97C_CBEVT     (0x1 << 4 ) /**< (AC97C)  */
/* --- Register AC97C_IER */
#define AT91C_AC97C_SOF       (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_WKUP      (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_COEVT     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_CAEVT     (0x1 << 3 ) /**< (AC97C)  */
#define AT91C_AC97C_CBEVT     (0x1 << 4 ) /**< (AC97C)  */
/* --- Register AC97C_IDR */
#define AT91C_AC97C_SOF       (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_WKUP      (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_COEVT     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_CAEVT     (0x1 << 3 ) /**< (AC97C)  */
#define AT91C_AC97C_CBEVT     (0x1 << 4 ) /**< (AC97C)  */
/* --- Register AC97C_IMR */
#define AT91C_AC97C_SOF       (0x1 << 0 ) /**< (AC97C)  */
#define AT91C_AC97C_WKUP      (0x1 << 1 ) /**< (AC97C)  */
#define AT91C_AC97C_COEVT     (0x1 << 2 ) /**< (AC97C)  */
#define AT91C_AC97C_CAEVT     (0x1 << 3 ) /**< (AC97C)  */
#define AT91C_AC97C_CBEVT     (0x1 << 4 ) /**< (AC97C)  */

#endif /* __AT91_AC97C_H */
