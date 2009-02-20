/*
 * Register definitions for Nordic nRF24L01
 *
 * Matthew Dombroski
 * Copyright (C) 2009 4D Electronics LTD (NZ)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_NRF24L01_H__
#define __LINUX_NRF24L01_H__

/* platform data definition */
struct nrf24l01_platform_data {
	void (*set_rf_enable) (int state);
};

/* packet structure */
struct nrf_packet {
	/* destination/source address */
	/* addr[0] is LSB */
	char addr[5];

	/* packet data */
	char *buf;

	/* length of data */
	int len;

	/* wants an ack packet in reply */
	int ack;
};

/* ack packet */
struct nrf_ack {
	/* packet data */
	char *buf;
	
	/* length of data */
	int len;
	
	/* destination pipe */
	int pipe;
};

/* Data pipe configuration */
struct nrf_pipe {
	/* pipe number (0-5) */
	int num;

	/* receive address */
	/* addr[0] is LSB */
	char addr[5];

	/* enable receive on this pipe */
	int enable:1;

	/* enable auto acknowledge on this pipe */
	int ack:1;

	/* enable dynamic payload length on this pipe */
	int dynpd:1;

	/* length of received packets (not for dynamic length) */
	int len;
};

/* Radio configuration */
struct nrf_radio_config {
	/* bytes of CRC */
	int crc;

	/* address width */
	int awidth;

	/* Delay between retransmission (in units of 250us) */
	int delay;

	/* retries */
	int retries;

	/* Radio channel */
	int channel;

	/* data rate */
	int rate;

	/* power output */
	int power;

	/* transmit address */
	char txaddr[5];
};

/* Register contents */
struct nrf_reg {
	char reg;
	char* data;
	int len;
};


/*
 * Definitions
 */

#define TX_SUCCESS	0
#define TX_MAX_RT	1
#define TX_FULL		2

#define STATE_POWER_DN	0
#define STATE_STANDBY	1
#define STATE_RX	2
#define STATE_TX	3

#define NRF_PLD_NO_ACK	0
#define NRF_PLD_NORMAL	1

/*
 * IOCTL definitions
 */

#define NRF_IOC_MAGIC 'x'

#define NRF_IOC_RESET _IO(NRF_IOC_MAGIC, 1)

#define NRF_IOC_FLUSH_RX _IO(NRF_IOC_MAGIC, 2)
#define NRF_IOC_FLUSH_TX _IO(NRF_IOC_MAGIC, 3)

#define NRF_IOC_TX_PACKET _IOW(NRF_IOC_MAGIC, 4, struct nrf_packet)
#define NRF_IOC_RX_PACKET _IOR(NRF_IOC_MAGIC, 5, struct nrf_packet)
#define NRF_IOC_ACK_PACKET _IOW(NRF_IOC_MAGIC, 6, struct nrf_ack)

#define NRF_IOC_CONFIG_PIPE _IOW(NRF_IOC_MAGIC, 7, struct nrf_pipe)
#define NRF_IOC_CONFIG_RADIO _IOW(NRF_IOC_MAGIC, 8, struct nrf_radio_config)

#define NRF_IOC_MODE_PWR_DOWN _IO(NRF_IOC_MAGIC, 9)
#define NRF_IOC_MODE_STANDBY1 _IO(NRF_IOC_MAGIC, 10)
#define NRF_IOC_MODE_TX _IO(NRF_IOC_MAGIC, 11)
#define NRF_IOC_MODE_RX _IO(NRF_IOC_MAGIC, 12)

/* please dont use these often - they are to be used for testing. */
#define NRF_IOC_READ_REG _IOR(NRF_IOC_MAGIC, 13, struct nrf_reg)
#define NRF_IOC_WRITE_REG _IOR(NRF_IOC_MAGIC, 14, struct nrf_reg)

#define NRF_IOC_MAX 14

/*
 * Register definitions for Nordic nRF24L01
 */

/* Definitions */
#define MAX_TX_BYTES			32

/* Commands */
#define CMD_R_REGISTER			0x00
#define CMD_W_REGISTER			0x20
#define CMD_R_RX_PAYLOAD		0x61
#define CMD_W_TX_PAYLOAD		0xA0
#define CMD_FLUSH_TX			0xE1
#define CMD_FLUSH_RX			0xE2
#define CMD_REUSE_TX_PL			0xE3
#define CMD_ACTIVATE			0x50
#define CMD_R_RX_PL_WID			0x60
#define CMD_W_ACK_PAYLOAD		0xA8
#define CMD_W_TX_PAYLOAD_NO_ACK		0xB0
#define CMD_NOP				0xFF

/* Register offsets */
#define REG_CONFIG			0x00
#define REG_EN_AA			0x01
#define REG_EN_RXADDR			0x02
#define REG_SETUP_AW			0x03
#define REG_SETUP_RETR			0x04
#define REG_RF_CH			0x05
#define REG_RF_SETUP			0x06
#define REG_STATUS			0x07
#define REG_OBSERVE_TX			0x08
#define REG_CD				0x09
#define REG_RX_ADDR_P0			0x0A	/* 5 bytes of address */
#define REG_RX_ADDR_P1			0x0B	/* 5 bytes of address */
#define REG_RX_ADDR_P2			0x0C	/* 1 byte of address (4 bytes from P1) */
#define REG_RX_ADDR_P3			0x0D	/* 1 byte of address (4 bytes from P1) */
#define REG_RX_ADDR_P4			0x0E	/* 1 byte of address (4 bytes from P1) */
#define REG_RX_ADDR_P5			0x0F	/* 1 byte of address (4 bytes from P1) */
#define REG_TX_ADDR			0x10
#define REG_RX_PW_P0			0x11
#define REG_RX_PW_P1			0x12
#define REG_RX_PW_P2			0x13
#define REG_RX_PW_P3			0x14
#define REG_RX_PW_P4			0x15
#define REG_RX_PW_P5			0x16
#define REG_FIFO_STATUS			0x17
#define REG_DYNPD			0x1C
#define REG_FEATURE			0x1D

/* Bitfields in Config register */
#define BIT_CONFIG_MASK_RX_DR		0x40
#define BIT_CONFIG_MASK_TX_DS		0x20
#define BIT_CONFIG_MASK_MAX_RT		0x10
#define BIT_CONFIG_EN_CRC		0x08
#define BIT_CONFIG_CRCO			0x04
#define BIT_CONFIG_PWR_UP		0x02
#define BIT_CONFIG_PRIM_RX		0x01

/* Bitfields in RF Setup register */
#define BIT_RFSETUP_RF_DR_LOW		0x20
#define BIT_RFSETUP_PLL_LOCK		0x10
#define BIT_RFSETUP_RF_DR_HIGH		0x08
#define BIT_RFSETUP_RF_PWR_1		0x02
#define BIT_RFSETUP_RF_PWR_2		0x04
#define BIT_RFSETUP_LNA_HCURR		0x01	/* obsolete */

/* Bitfields in status register */
#define BIT_STATUS_RX_DR		0x40
#define BIT_STATUS_TX_DS		0x20
#define BIT_STATUS_MAX_RT		0x10
#define BIT_STATUS_TX_FULL		0x01
#define BIT_STATUS_MASK_RX_PIPE		0x0E

/* Bitfields in FIFO status register */
#define BIT_FIFO_TX_REUSE		0x40
#define BIT_FIFO_TX_FULL		0x20
#define BIT_FIFO_TX_EMPTY		0x10
#define BIT_FIFO_RX_FULL		0x02
#define BIT_FIFO_RX_EMPTY		0x01

/* Bitfields in Feature register */
#define BIT_FEATURE_EN_DPL		0x04
#define BIT_FEATURE_EN_ACK_PAY		0x02
#define BIT_FEATURE_EN_DYN_ACK		0x01

#endif	/* __LINUX_NRF24L01_H__ */

