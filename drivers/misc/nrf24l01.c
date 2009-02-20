/*
 * Nordic nRF24L01 driver
 *
 * Matthew Dombroski
 * Copyright (C) 2009 4D Electronics LTD (NZ)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/nrf24l01.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/ioctl.h>

#define DEVICE_NAME "nrf24l01"

/* structure used to get information to the file operation functions */
/* and to keep some state information. */
struct nrf24l01 {
	/* configuration of the radio */
	struct nrf_pipe pipes[6];
	struct nrf_radio_config radio;

	struct spi_device *spi;

	/* platform data from board configuration */
	struct nrf24l01_platform_data *pdata;

	/* to ensure that the character device can only be opened by one app */
	/* concurrently */
	struct mutex file_mutex;

	/* for locking interrupts in the isr and the suspend/resume functions */
	spinlock_t lock;

	/* set on interrupt */
	struct mutex interrupted;

	/* unocked when a packet is received */
	struct mutex rx;

	/* unocked when a packet is transmitted */
	struct mutex tx;

	/* when servicing the interrupt the status of the tx packet is placed */
	/* here */
	int tx_status;

	/* power state */
	int state;
};

/* structure used in probe/remove */
struct nrf_dev {
	struct nrf24l01 *dev;
	int nrf_major;
	struct class *nrf_class;
};

static struct nrf_dev nrf_static_dev = {
	.nrf_major = 0,
};


/*
 * Driver functionality
 */

/* Perform SPI read command with nRF opcode */
static int nrf_read(struct spi_device *spi, uint8_t opcode, char *buf, int len)
{
	int err = 0;
	int status = 0;

	struct spi_message m;
	struct spi_transfer t[2] = {
		{
			.tx_buf = &opcode,
			.rx_buf = &status,
			.len = 1,
			.cs_change = 0,
		},
		{
			.rx_buf = buf,
			.len = len,
			.cs_change = 0,
		}
	};

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	err = spi_sync(spi, &m);

	return (err < 0) ? err : status;
}

/* Perform SPI write command with nRF opcode */
static int nrf_write(struct spi_device *spi, uint8_t opcode, const char *buf,
		     int len)
{
	int err = 0;
	int status = 0;

	struct spi_message m;
	struct spi_transfer t[2] = {
		{
			.tx_buf = &opcode,
			.rx_buf = &status,
			.len = 1,
			.cs_change = 0,
		},
		{
			.tx_buf = buf,
			.len = len,
			.cs_change = 0,
		}
	};

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	err = spi_sync(spi, &m);

	return (err < 0) ? err : status;
}

/* Read the status register (with NOP command) */
static inline int nrf_status(struct spi_device *spi)
{
	return nrf_read(spi, CMD_NOP, 0x00, 0x00);
}

/* Read contents of a register into a buffer. some registers have several bytes */
static inline int nrf_read_register(struct spi_device *spi, uint8_t reg,
				    char *buf, int len)
{
	return nrf_read(spi, reg | CMD_R_REGISTER, buf, len);
}

/* Write contents of a register from a buffer. some registers have several bytes */
static inline int nrf_write_register(struct spi_device *spi, uint8_t reg,
				     const char *buf, int len)
{
	return nrf_write(spi, reg | CMD_W_REGISTER, buf, len);
}

/* Get the width of the top packet in the receive fifo */
static inline int nrf_read_payload_width(struct spi_device *spi)
{
	char width;
	int err;
	err = nrf_read(spi, CMD_R_RX_PL_WID, &width, 1);
	return (err < 0) ? err : width;
}

/* Read payload out of rx fifo */
static inline int nrf_read_payload(struct spi_device *spi, char *buf, int len)
{
	return nrf_read(spi, CMD_R_RX_PAYLOAD, buf, len);
}

/* Write payload to tx fifo */
static inline int nrf_write_payload(struct spi_device *spi, const char *buf,
				    int len)
{
	return nrf_write(spi, CMD_W_TX_PAYLOAD, buf, len);
}

/* Write payload to tx fifo. This packet will not expect an ACK in reply */
static inline int nrf_write_payload_noack(struct spi_device *spi,
					  const char *buf, int len)
{
	return nrf_write(spi, CMD_W_TX_PAYLOAD_NO_ACK, buf, len);
}

/* Write payload to ACK FIFO. */
static inline int nrf_write_ack_payload(struct spi_device *spi, int pipe,
					  const char *buf, int len)
{
	return nrf_write(spi, CMD_W_ACK_PAYLOAD | pipe, buf, len);
}

/* Flush the tx fifo buffers - use when transmit fails */
static inline int nrf_flush_tx(struct nrf24l01 *nrf)
{
	/* lock tx mutex as packets are no longer in fifo */
	mutex_trylock(&nrf->tx);
	return nrf_write(nrf->spi, CMD_FLUSH_TX, 0x00, 0x00);
}

/* Flush the rx fifo buffers */
static inline int nrf_flush_rx(struct nrf24l01 *nrf)
{
	/* lock rx mutex as packets are no longer available */
	mutex_trylock(&nrf->rx);
	return nrf_write(nrf->spi, CMD_FLUSH_RX, 0x00, 0x00);
}

/* Put the radio into power down mode */
static int nrf_mode_power_down(struct nrf24l01 *nrf)
{
	int status;
	char reg;

	/* write PWR_UP=0 to config register */
	status = nrf_read_register(nrf->spi, REG_CONFIG, &reg, 1);
	reg &= ~BIT_CONFIG_PWR_UP;
	status = nrf_write_register(nrf->spi, REG_CONFIG, &reg, 1);
	nrf->pdata->set_rf_enable(0);
	nrf->state = STATE_POWER_DN;

	return status;
}

/* Put the radio into standby mode */
static int nrf_mode_standby(struct nrf24l01 *nrf)
{
	int status;
	char reg;

	status = nrf_read_register(nrf->spi, REG_CONFIG, &reg, 1);
	reg |= BIT_CONFIG_PWR_UP;
	status = nrf_write_register(nrf->spi, REG_CONFIG, &reg, 1);
	nrf->pdata->set_rf_enable(0);
	nrf->state = STATE_STANDBY;

	return status;
}

/* Put the radio into tx mode */
static int nrf_mode_tx(struct nrf24l01 *nrf)
{
	int status;
	char reg;

	status = nrf_read_register(nrf->spi, REG_CONFIG, &reg, 1);
	reg |= BIT_CONFIG_PWR_UP;
	reg &= ~BIT_CONFIG_PRIM_RX;
	status = nrf_write_register(nrf->spi, REG_CONFIG, &reg, 1);
	nrf->pdata->set_rf_enable(1);
	nrf->state = STATE_TX;

	return status;
}

/* Put the radio into rx mode */
static int nrf_mode_rx(struct nrf24l01 *nrf)
{
	int status;
	char reg;

	status = nrf_read_register(nrf->spi, REG_CONFIG, &reg, 1);
	reg |= BIT_CONFIG_PWR_UP | BIT_CONFIG_PRIM_RX;
	status = nrf_write_register(nrf->spi, REG_CONFIG, &reg, 1);
	nrf->pdata->set_rf_enable(1);
	nrf->state = STATE_RX;

	return status;
}

/* Configure a data pipe */
static int nrf_configure_pipe(struct nrf24l01 *nrf, struct nrf_pipe *pipe)
{
	int status = 0;
	char buf, width;

	if ((pipe->len > 32) | (pipe->num > 5))
		return -EINVAL;

	/* get address width configuration. */
	if (pipe->num < 2)
		/* pipes 0, 1 have max 5 bytes address */
		width = nrf->radio.awidth;
	else
		/* pipes 2 - 5 have 1 byte MSB */
		width = 1;

	/* configure pipe address */
	status |=
	    nrf_write_register(nrf->spi, REG_RX_ADDR_P0 + pipe->num, pipe->addr,
			       width);

	/* enable/disable pipe */
	status |= nrf_read_register(nrf->spi, REG_EN_RXADDR, &buf, 1);
	if (pipe->enable)
		buf |= (1 << pipe->num);
	else
		buf &= ~(1 << pipe->num);
	status |= nrf_write_register(nrf->spi, REG_EN_RXADDR, &buf, 1);

	/* enable/disable ack */
	status |= nrf_read_register(nrf->spi, REG_EN_AA, &buf, 1);
	if (pipe->ack)
		buf |= (1 << pipe->num);
	else
		buf &= ~(1 << pipe->num);
	status |= nrf_write_register(nrf->spi, REG_EN_AA, &buf, 1);

	/* enable/disable dynamic payload */
	status |= nrf_read_register(nrf->spi, REG_DYNPD, &buf, 1);
	if (pipe->dynpd)
		buf |= (1 << pipe->num);
	else
		buf &= ~(1 << pipe->num);
	status |= nrf_write_register(nrf->spi, REG_DYNPD, &buf, 1);

	buf = pipe->len;
	status |=
	    nrf_write_register(nrf->spi, REG_RX_PW_P0 + pipe->num, &buf, 1);

	return (status < 0) ? -EIO : 0;
}

/* Get the configuration of a data pipe. Configuration is stored in nrf struct */
static int nrf_configure_pipe_get(struct nrf24l01 *nrf, int num)
{
	char buf;
	int status = 0, width;
	struct nrf_pipe *pipe = &nrf->pipes[num];
	
	if (num > 5)
		return -EINVAL;

	pipe->num = num;

	/* get address width configuration. */
	if (pipe->num < 2)
		/* pipes 0, 1 have max 5 bytes address */
		width = nrf->radio.awidth;
	else
		/* pipes 2 - 5 have 1 byte MSB */
		width = 1;

	/* configure pipe address */
	status |=
	    nrf_read_register(nrf->spi, REG_RX_ADDR_P0 + num, pipe->addr,
			      width);

	/* enable/disable pipe */
	status |= nrf_read_register(nrf->spi, REG_EN_RXADDR, &buf, 1);
	pipe->enable = (buf & (1 << pipe->num)) ? 1 : 0;

	/* enable/disable ack */
	status |= nrf_read_register(nrf->spi, REG_EN_AA, &buf, 1);
	pipe->ack = (buf & (1 << pipe->num)) ? 1 : 0;

	/* enable/disable dynamic payload */
	status |= nrf_read_register(nrf->spi, REG_DYNPD, &buf, 1);
	pipe->dynpd = (buf & (1 << pipe->num)) ? 1 : 0;

	status |= nrf_read_register(nrf->spi, REG_RX_PW_P0 + num, &buf, 1);
	pipe->len = buf;

	return 0;
}

/* Set the radio configuration parameters */
static int nrf_configure_radio(struct nrf24l01 *nrf,
			       struct nrf_radio_config *radio)
{
	int status = 0;
	char buf;

	/* invalid parameters */
	if ((radio->crc < 0x00) || (radio->crc > 0x02))
		return -EINVAL;

	if ((radio->awidth < 0x03) || (radio->awidth > 0x05))
		return -EINVAL;

	if ((radio->delay < 0x00) || (radio->delay > 0x0F))
		return -EINVAL;

	if ((radio->retries < 0x00) || (radio->retries > 0x0F))
		return -EINVAL;

	if ((radio->channel < 0x00) || (radio->channel > 0x7F))
		return -EINVAL;

	if ((radio->rate < 0x00) || (radio->rate > 0x02))
		return -EINVAL;

	if ((radio->power < 0x00) || (radio->power > 0x03))
		return -EINVAL;

	/* configure CRC */
	status |= nrf_read_register(nrf->spi, REG_CONFIG, &buf, 1);
	switch (radio->crc) {
	case 2:
		buf |= BIT_CONFIG_CRCO | BIT_CONFIG_EN_CRC;
		break;
	case 1:
		buf |= BIT_CONFIG_EN_CRC;
		buf &= ~BIT_CONFIG_CRCO;
		break;
	case 0:
		buf &= ~(BIT_CONFIG_CRCO | BIT_CONFIG_EN_CRC);
		break;
	}
	status |= nrf_write_register(nrf->spi, REG_CONFIG, &buf, 1);

	/* configure address width */
	buf = radio->awidth - 2;
	status |= nrf_write_register(nrf->spi, REG_SETUP_AW, &buf, 1);

	/* configure delay and retries */
	buf = (radio->delay << 4) | radio->retries;
	status |= nrf_write_register(nrf->spi, REG_SETUP_RETR, &buf, 1);

	/* configure radio channel */
	buf = radio->channel;
	status |= nrf_write_register(nrf->spi, REG_RF_CH, &buf, 1);

	/* configure data rate and power */
	status |= nrf_read_register(nrf->spi, REG_RF_SETUP, &buf, 1);
	switch (radio->rate) {
	case 2:
		buf &= ~BIT_RFSETUP_RF_DR_LOW;
		buf |= BIT_RFSETUP_RF_DR_HIGH;
		break;
	case 1:
		buf &= ~(BIT_RFSETUP_RF_DR_LOW | BIT_RFSETUP_RF_DR_HIGH);
		break;
	case 0:
		buf |= BIT_RFSETUP_RF_DR_LOW;
		buf &= ~BIT_RFSETUP_RF_DR_HIGH;
		break;
	}
	buf |= (radio->power << 1);
	status |= nrf_write_register(nrf->spi, REG_RF_SETUP, &buf, 1);

	/* set tx address */
	status |=
	    nrf_write_register(nrf->spi, REG_TX_ADDR, radio->txaddr,
			       radio->awidth);

	return (status < 0) ? -EIO : 0;
}

/* Get the radio configuration parameters. Configuration is stored in nrf struct */
static int nrf_configure_radio_get(struct nrf24l01 *nrf)
{
	struct nrf_radio_config *radio = &nrf->radio;
	int status = 0;
	char buf;

	/* get CRC configuration */
	status |= nrf_read_register(nrf->spi, REG_CONFIG, &buf, 1);
	switch (buf & (BIT_CONFIG_CRCO | BIT_CONFIG_EN_CRC)) {
	case BIT_CONFIG_CRCO | BIT_CONFIG_EN_CRC:
		radio->crc = 2;
		break;
	case BIT_CONFIG_EN_CRC:
		radio->crc = 1;
		break;
	case 0:
		radio->crc = 0;
		break;
	}

	/* get address width */
	status |= nrf_read_register(nrf->spi, REG_SETUP_AW, &buf, 1);
	radio->awidth = buf + 2;

	/* get delay and retries */
	status |= nrf_read_register(nrf->spi, REG_SETUP_RETR, &buf, 1);
	radio->delay = buf >> 4;
	radio->retries = buf & 0x0F;

	/* configure radio channel */
	status |= nrf_read_register(nrf->spi, REG_RF_CH, &buf, 1);
	radio->channel = buf;

	/* configure data rate and power */
	status |= nrf_read_register(nrf->spi, REG_RF_SETUP, &buf, 1);
	switch (buf & (BIT_RFSETUP_RF_DR_LOW | BIT_RFSETUP_RF_DR_HIGH)) {
	case BIT_RFSETUP_RF_DR_HIGH:
		radio->rate = 2;
		break;
	case 0:
		radio->rate = 1;
		break;
	case BIT_RFSETUP_RF_DR_LOW:
		radio->rate = 0;
		break;
	}
	radio->power = (buf >> 1) & 0x03;

	/* get tx address */
	status |= nrf_read_register(nrf->spi, REG_TX_ADDR, radio->txaddr,
				    radio->awidth);

	return (status < 0) ? -EIO : 0;
}

/* Service interrupt */
static int nrf_service_irq(struct nrf24l01 *nrf)
{
	int status;
	char flags;

	dev_dbg(&nrf->spi->dev, "Service IRQ\n");
	msleep(1);

	if (mutex_lock_interruptible(&nrf->interrupted)) {
		/* interrupted while waiting for lock */
		return -EINTR;
	}

	status = nrf_status(nrf->spi);
	flags =
	    status & (BIT_STATUS_RX_DR | BIT_STATUS_TX_DS | BIT_STATUS_MAX_RT |
		      BIT_STATUS_TX_FULL);
	status = nrf_write_register(nrf->spi, REG_STATUS, &flags, 1);

	if (flags & BIT_STATUS_RX_DR) {
		/* data received */
		mutex_unlock(&nrf->rx);
	}

	if (flags & BIT_STATUS_TX_DS) {
		/* data sent */
		mutex_unlock(&nrf->tx);
		nrf->tx_status = TX_SUCCESS;
	}

	if (flags & BIT_STATUS_MAX_RT) {
		/* max retries exceeded - failed to send packet */
		mutex_unlock(&nrf->tx);
		nrf->tx_status = TX_MAX_RT;
	}

	if (flags & BIT_STATUS_TX_FULL) {
		/* TX FIFO full */
		mutex_unlock(&nrf->tx);
		nrf->tx_status = TX_FULL;
	}

	enable_irq(nrf->spi->irq);

	return status;
}

/* The ISR, sets unlocks the interrupted mutex */
static irqreturn_t nrf_irq(int irq, void *handle)
{
	struct nrf24l01 *nrf = handle;
	unsigned long flags = 0;

	dev_dbg(&nrf->spi->dev, "IRQ %d\n", irq);

	spin_lock_irqsave(&nrf->lock, flags);
	disable_irq_nosync(irq);
	mutex_unlock(&nrf->interrupted);
	spin_unlock_irqrestore(&nrf->lock, flags);

	return IRQ_HANDLED;
}


static int nrf_tx_packet(struct nrf24l01 *nrf, const struct nrf_packet *packet)
{
	char state = 0;
	int err = 0;

	/* save current state */
	nrf_read_register(nrf->spi, REG_CONFIG, &state, 1);
	nrf_mode_standby(nrf);

	/* set tx address */
	nrf_write_register(nrf->spi, REG_TX_ADDR, packet->addr,
			   nrf->radio.awidth);

	/* copy buffer into FIFO */
	switch (packet->ack) {
	case NRF_PLD_NO_ACK:
		nrf_write_payload_noack(nrf->spi, packet->buf, packet->len);
		break;

	default:
	case NRF_PLD_NORMAL:
		nrf_write_payload(nrf->spi, packet->buf, packet->len);
		break;
	}

	/* tx packet */
	nrf_mode_tx(nrf);

	/* wait for interrupt */
	nrf_service_irq(nrf);

	/* wait for packet transmission to end - may be success or fail */
	if (mutex_lock_interruptible(&nrf->tx)) {
		/* flush tx fifo if interrupted before transmission complete */
		nrf_flush_tx(nrf);
		
		/* restore state */
		nrf_write_register(nrf->spi, REG_CONFIG, &state, 1);
		return -EINTR;
	}

	switch (nrf->tx_status) {
	case TX_SUCCESS:
		break;

	case TX_MAX_RT:
		nrf_flush_tx(nrf);
		dev_dbg(&nrf->spi->dev, "MAX_RT\n");
		err = -EAGAIN;
		break;
	case TX_FULL:
		nrf_flush_tx(nrf);
		dev_dbg(&nrf->spi->dev, "TX FIFO full\n");
		err = -EAGAIN;
		break;
	default:
		err = -EAGAIN;
		break;
	}

	/* restore state */
	nrf_write_register(nrf->spi, REG_CONFIG, &state, 1);
	
	return err;
}

static int nrf_rx_packet(struct nrf24l01 *nrf, struct nrf_packet *packet)
{
	char fifo;
	int pipe, status, width;

	dev_dbg(&nrf->spi->dev, "%s\n", __FUNCTION__);

	/* enter receive mode and wait for packet */
	nrf_mode_rx(nrf);
	while (1) {
		if (mutex_trylock(&nrf->rx))
			break;

		/* nrf_service_irq is interruptible so this infinite loop is acceptable */
		if (nrf_service_irq(nrf) < 0)
			return -EINTR;
	}
	
	status = nrf_read_register(nrf->spi, REG_FIFO_STATUS, &fifo, 1);
	if (status < 0)
		return -EIO;

	/* if RX FIFO is empty then error */
	if (fifo & BIT_FIFO_RX_EMPTY)
		return -EAGAIN;

	width = nrf_read_payload_width(nrf->spi);

	/* On invalid packet length flush rx */
	if ((width < 0) || (width > 32))
	{
		nrf_flush_rx(nrf);
		return -EIO;
	}

	packet->len = width;

	/* read out packet contents */
	if (nrf_read_payload(nrf->spi, packet->buf, width) < 0)
		return -EIO;

	/* get address of packet */
	pipe = (status & 0x0E) >> 1;
	if (pipe < 2) {
		nrf_read_register(nrf->spi, REG_RX_ADDR_P0 + pipe, packet->addr,
				  5);
	} else {
		nrf_read_register(nrf->spi, REG_RX_ADDR_P1, packet->addr, 5);
		nrf_read_register(nrf->spi, REG_RX_ADDR_P0 + pipe,
				  packet->addr, 1);
	}

	/* if there are still packets to receive unlock the mutex */
	nrf_read_register(nrf->spi, REG_FIFO_STATUS, &fifo, 1);
	if (!(fifo & BIT_FIFO_RX_EMPTY))
		mutex_unlock(&nrf->rx);
	
	return 0;
}

static int nrf_ack_packet(struct nrf24l01 *nrf, struct nrf_ack *packet)
{
	int status;
	
	if ((packet->pipe < 0) || (packet->pipe > 5))
		return -EINVAL;

	status = nrf_write_ack_payload(nrf->spi, packet->pipe, packet->buf, packet->len);
	
/*	if (status) */
		return 0;
/*	else
		return -EIO; */
}


/*
 * Character device operations
 */

static int nrf_file_open(struct inode *ip, struct file *fp)
{
	/* maybe increment the use count */
	/* even block if already opened? */
	struct nrf24l01 *nrf = nrf_static_dev.dev;

	if (!mutex_trylock(&nrf->file_mutex))
		return -EBUSY;

	fp->private_data = nrf;

	/* flush FIFO buffers */
	nrf_flush_rx(nrf);

	/* flush FIFO buffers */
	nrf_flush_tx(nrf);

	/* put nRF into standby-I mode */
	nrf_mode_standby(nrf);

	return 0;
}

static int nrf_file_release(struct inode *ip, struct file *fp)
{
	struct nrf24l01 *nrf = fp->private_data;

	/* put nRF into low power off mode */
	nrf_mode_power_down(nrf);

	mutex_unlock(&nrf->file_mutex);
	return 0;
}

/* copy 1 rx packet to user buffer */
static ssize_t nrf_file_read(struct file *fp, char *buf, size_t len,
			     loff_t * off)
{
	struct nrf24l01 *nrf = fp->private_data;
	char buffer[32];
	char fifo;
	int i, width, read_count, status;

	/* enter receive mode and wait for packet */
	nrf_mode_rx(nrf);
	while (1) {
		if (mutex_trylock(&nrf->rx))
			break;

		/* nrf_service_irq is interruptible so this infinite loop is acceptable */
		if (nrf_service_irq(nrf) < 0)
			return -EINTR;
	}

	read_count = 0;
	/* 10 is arbitrary. this was originally an infinite loop, bounded for safety */
	for (i = 0; i < 10; i++) {
		status = nrf_read_register(nrf->spi, REG_FIFO_STATUS, &fifo, 1);
		if (status < 0)
			return -EIO;

		/* if RX FIFO is empty then break loop */
		if (fifo & BIT_FIFO_RX_EMPTY)
			break;

		width = nrf_read_payload_width(nrf->spi);
		if ((width < 0) || (width > 32))
			return -EIO;

		if (nrf_read_payload(nrf->spi, buffer, width) < 0)
			return -EIO;

		width =
		    (width < (len - read_count)) ? width : (len - read_count);

		if (copy_to_user(&buf[read_count], buffer, width))
			return -EFAULT;

		read_count += width;
	}

	/* if there are still packets to receive unlock the mutex */
	nrf_read_register(nrf->spi, REG_FIFO_STATUS, &fifo, 1);
	if (!(fifo & BIT_FIFO_RX_EMPTY))
		mutex_unlock(&nrf->rx);

	return read_count;
}

/* fill the tx fifo (3 levels) and transmit */
static ssize_t nrf_file_write(struct file *fp, const char *buf, size_t len,
			      loff_t * off)
{
	struct nrf24l01 *nrf = fp->private_data;
	char buffer[32];
	size_t copied = 0;
	int i;
	int max;
	char reg;
	char state = 0;
	
	/* save current state */
	nrf_read_register(nrf->spi, REG_CONFIG, &state, 1);

	if (len == 0) {
		return -EINVAL;
	}

	nrf_mode_standby(nrf);

	/* maximum transfer length is 32 */
	/* can queue 3 transfers at a time */
	max = (len < 96) ? len : 96;
	for (copied = 0; copied < max; copied += i) {
		/* If the tx fifo is full bail out */
		nrf_read_register(nrf->spi, REG_FIFO_STATUS, &reg, 1);
		if (reg & BIT_FIFO_TX_FULL)
			break;

		i = ((len - copied) < 32) ? (len - copied) : 32;
		if (copy_from_user(buffer, &buf[copied], i)) {
			nrf_flush_tx(nrf);
			return -EFAULT;
		}

		nrf_write_payload(nrf->spi, buffer, i);
	}

	nrf_mode_tx(nrf);

	/* wait for interrupt */
	nrf_service_irq(nrf);

	/* wait for packet transmission to end - may be success or fail */
	if (mutex_lock_interruptible(&nrf->tx)) {
		/* clear tx fifo on interrupt before packet transmission complete */
		nrf_flush_tx(nrf);
		
		/* restore state */
		nrf_write_register(nrf->spi, REG_CONFIG, &state, 1);
		
		return -EINTR;
	}

	switch (nrf->tx_status) {
	case TX_SUCCESS:
		break;

	case TX_MAX_RT:
		nrf_flush_tx(nrf);
		dev_dbg(&nrf->spi->dev, "MAX_RT\n");
		copied = -EAGAIN;
		break;
	case TX_FULL:
		nrf_flush_tx(nrf);
		dev_dbg(&nrf->spi->dev, "TX FIFO full\n");
		copied = -EAGAIN;
		break;
	default:
		copied = -EAGAIN;
		break;
	}

	/* restore state */
	nrf_write_register(nrf->spi, REG_CONFIG, &state, 1);
	
	return copied;
}

static int nrf_file_ioctl(struct inode *ip, struct file *fp, unsigned int cmd,
			  unsigned long arg)
{
	struct nrf24l01 *nrf = fp->private_data;
	struct nrf_pipe *pipe;
	struct nrf_radio_config *radio, *radio_user;
	struct nrf_packet *packet, *packet_user;
	struct nrf_ack *ack, *ack_user;
	struct nrf_reg *reg_user;
	char* buf;
	unsigned long tmp;
	int err = 0;

	if (_IOC_TYPE(cmd) != NRF_IOC_MAGIC) {
		dev_err(&nrf->spi->dev, "invalid ioctl (magic)\n");
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > NRF_IOC_MAX) {
		dev_err(&nrf->spi->dev, "invalid ioctl (max)\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case NRF_IOC_RESET:
		nrf_service_irq(nrf);
		nrf_flush_tx(nrf);
		nrf_flush_rx(nrf);
		break;

	case NRF_IOC_FLUSH_RX:
		nrf_flush_rx(nrf);
		break;

	case NRF_IOC_FLUSH_TX:
		nrf_flush_tx(nrf);
		break;

	case NRF_IOC_TX_PACKET:
		packet_user = (struct nrf_packet *)arg;

		packet = kzalloc(sizeof(struct nrf_packet), GFP_KERNEL);
		if (packet == NULL)
			return -EFAULT;

		if (copy_from_user
		    (packet, packet_user, sizeof(struct nrf_packet))) {
			kfree(packet);
			return -EFAULT;
		}

		packet->buf =
		    kzalloc(sizeof(char) * packet_user->len, GFP_KERNEL);
		if (packet->buf == NULL)
			return -EFAULT;
		if (copy_from_user
		    (packet->buf, packet_user->buf, packet_user->len)) {
			kfree(packet->buf);
			kfree(packet);
			return -EFAULT;
		}

		err = nrf_tx_packet(nrf, packet);

		kfree(packet->buf);
		kfree(packet);
		break;

	case NRF_IOC_RX_PACKET:
		packet_user = (struct nrf_packet *)arg;

		packet = kzalloc(sizeof(struct nrf_packet), GFP_KERNEL);
		if (packet == NULL)
			return -EFAULT;

		/* allocate full 32 byte buffer */
		packet->buf = kzalloc(sizeof(char) * 32, GFP_KERNEL);
		if (packet->buf == NULL) {
			kfree(packet);
			return -EFAULT;
		}

		err = nrf_rx_packet(nrf, packet);
		if( err ) {
			kfree(packet->buf);
			kfree(packet);
			return err;
		}

		/* copy the packet contents - address, size */
		/* unfortunately this kills the buf pointer, so back it up */
		tmp = (unsigned long)packet_user->buf;
		if (copy_to_user
		    (packet_user, packet, sizeof(struct nrf_packet))) {
			kfree(packet->buf);
			kfree(packet);
			return -EFAULT;
		}
		/* restore the buf pointer */
		packet_user->buf = (char *)tmp;
		
		/* copy the packet buffer */
		err = copy_to_user(packet_user->buf, packet->buf, packet->len);
		if (err) {
			kfree(packet->buf);
			kfree(packet);
			return -EFAULT;
		}

		kfree(packet->buf);
		kfree(packet);
		break;

	case NRF_IOC_ACK_PACKET:
		ack_user = (struct nrf_ack *)arg;

		ack = kzalloc(sizeof(struct nrf_ack), GFP_KERNEL);
		if (ack == NULL)
			return -EFAULT;

		if (copy_from_user(ack, ack_user, sizeof(struct nrf_ack))) {
			kfree(ack);
			return -EFAULT;
		}

		ack->buf = kzalloc(sizeof(char) * ack_user->len, GFP_KERNEL);
		if (ack->buf == NULL)
			return -EFAULT;

		if (copy_from_user(ack->buf, ack_user->buf, ack_user->len)) {
			kfree(ack->buf);
			kfree(ack);
			return -EFAULT;
		}

		err = nrf_ack_packet(nrf, ack);

		kfree(ack->buf);
		kfree(ack);
		break;

	case NRF_IOC_CONFIG_PIPE:
		pipe = (struct nrf_pipe *)arg;
		if (copy_from_user
		    (&nrf->pipes[pipe->num], pipe, sizeof(struct nrf_pipe)))
			return -EFAULT;
		err = nrf_configure_pipe(nrf, &nrf->pipes[pipe->num]);
		break;

	case NRF_IOC_CONFIG_RADIO:
		radio = kzalloc(sizeof(struct nrf_radio_config), GFP_KERNEL);
		if (radio == NULL)
			return -EFAULT;

		radio_user = (struct nrf_radio_config *)arg;

		if (copy_from_user
		    (radio, radio_user, sizeof(struct nrf_radio_config))) {
			kfree(radio);
			return -EFAULT;
		}

		err = nrf_configure_radio(nrf, radio);
		if (err == 0)
			memcpy(&nrf->radio, radio,
				sizeof(struct nrf_radio_config));

		kfree(radio);
		break;

	case NRF_IOC_MODE_PWR_DOWN:
		nrf_mode_power_down(nrf);
		break;

	case NRF_IOC_MODE_STANDBY1:
		nrf_mode_standby(nrf);
		break;

	case NRF_IOC_MODE_TX:
		nrf_mode_tx(nrf);
		break;

	case NRF_IOC_MODE_RX:
		nrf_mode_rx(nrf);
		break;

	case NRF_IOC_READ_REG:
		reg_user = (struct nrf_reg *)arg;

		buf = kzalloc(sizeof(char)*reg_user->len, GFP_KERNEL);
		if (buf == NULL)
			return -EFAULT;

		err = nrf_read_register(nrf->spi, reg_user->reg, buf, reg_user->len);

		if (copy_to_user(reg_user->data, buf, reg_user->len)) {
			kfree(buf);
			return -EFAULT;
		}

		kfree(buf);
		break;

	case NRF_IOC_WRITE_REG:
		reg_user = (struct nrf_reg *)arg;

		buf = kzalloc(sizeof(char)*reg_user->len, GFP_KERNEL);
		if (buf == NULL)
			return -EFAULT;

		if (copy_from_user(buf, reg_user->data, reg_user->len)) {
			kfree(buf);
			return -EFAULT;
		}

		err = nrf_write_register(nrf->spi, reg_user->reg, buf, reg_user->len);

		kfree(buf);
		break;

	default:
		return -ENOTTY;
		break;
	}

	return err;
}

static struct file_operations nrf_ops = {
	.owner = THIS_MODULE,
	.read = nrf_file_read,
	.write = nrf_file_write,
	.open = nrf_file_open,
	.release = nrf_file_release,
	.ioctl = nrf_file_ioctl,
};


/*
 * SPI driver operations (probe, remove...)
 */

/* configure IRQ, memory, SPI, character device */
static int __devinit nrf_probe(struct spi_device *spi)
{
	struct nrf24l01_platform_data *pdata = spi->dev.platform_data;
	struct nrf24l01 *nrf;
	struct device *dev;
	char magic = 0x00;
	int err = 0;
	int i = 0;

	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_err(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}
	
	/* 10 Mbit/s is maximum */
	if (spi->max_speed_hz > 10 * 1000 * 1000) {
		dev_err(&spi->dev, "%d SPI clock?\n", spi->max_speed_hz);
		return -EINVAL;
	}

	if (pdata->set_rf_enable == NULL) {
		dev_err(&spi->dev, "no set_rf_enable function?\n");
		return -EINVAL;
	}

	nrf = kzalloc(sizeof(struct nrf24l01), GFP_KERNEL);
	if (nrf == NULL)
		return -EINVAL;

	nrf_static_dev.dev = nrf;
	dev_set_drvdata(&spi->dev, nrf);
	nrf->spi = spi;
	nrf->pdata = spi->dev.platform_data;
	nrf->tx_status = TX_SUCCESS;

	mutex_init(&nrf->file_mutex);

	mutex_init(&nrf->rx);
	mutex_trylock(&nrf->rx);

	mutex_init(&nrf->tx);
	mutex_trylock(&nrf->tx);

	mutex_init(&nrf->interrupted);
	mutex_trylock(&nrf->interrupted);

	spin_lock_init(&nrf->lock);

	/* 8 bit transfers, active high clock, transfer on mode 0 */
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0) {
		dev_err(&spi->dev, "Error configuring SPI\n");
		goto errmem;
	}
	
	/* make sure that we can read/write on spi */
	err = nrf_status(spi);
	if (err < 0) {
		
		dev_err(&spi->dev, "Error communicating on SPI\n");
		goto errmem;
	}

	err = request_irq(spi->irq, nrf_irq, IRQF_TRIGGER_FALLING,
			spi->dev.driver->name, nrf_static_dev.dev);
	if (err) {
		dev_err(&spi->dev, "irq %d busy?\n", spi->irq);
		err = -EBUSY;
		goto errirq;
	}
	dev_info(&spi->dev, "Assigned IRQ=%d\n", spi->irq);
	
	/* register device class (for sysfs) */
	nrf_static_dev.nrf_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(nrf_static_dev.nrf_class)) {
		err = PTR_ERR(nrf_static_dev.nrf_class);
		dev_err(&spi->dev, "Error creating class\n");
		goto errclass;
	}
	
	/* register character device (get major) */
	err = register_chrdev(0, DEVICE_NAME, &nrf_ops);
	if (err < 0) {
		dev_err(&spi->dev, "Error registering character device\n");
		goto errchr;
	}
	nrf_static_dev.nrf_major = err;

	/* create character device /dev/nrf */
	/* parent is spi device */
	dev = device_create(nrf_static_dev.nrf_class, &spi->dev,
			MKDEV(nrf_static_dev.nrf_major, 0), NULL, "nrf");
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		dev_err(&spi->dev, "Error creating device\n");
		goto errdev;
	}
	
	/* set low power mode */
	nrf_mode_power_down(nrf);

	for (i = 0; i < 5 && magic == 0x00; i++) {
		/* enable features registers */
		magic = 0x73;
		nrf_write(nrf->spi, CMD_ACTIVATE, &magic, 1);

		/* enable dynamic payload feature */
		magic = BIT_FEATURE_EN_DPL | BIT_FEATURE_EN_ACK_PAY | BIT_FEATURE_EN_DYN_ACK;
		nrf_write_register(nrf->spi, REG_FEATURE, &magic, 1);

		/* check dynamic payload is enabled */
		/* magic should be non-zero, causing the loop to end. */
		nrf_read_register(nrf->spi, REG_FEATURE, &magic, 1);
	}
	dev_info(&nrf->spi->dev, "feature register enabled\n");

	/* get configuration from radio */
	nrf_configure_radio_get(nrf);
	nrf_configure_pipe_get(nrf, 0);
	nrf_configure_pipe_get(nrf, 1);
	nrf_configure_pipe_get(nrf, 2);
	nrf_configure_pipe_get(nrf, 3);
	nrf_configure_pipe_get(nrf, 4);
	nrf_configure_pipe_get(nrf, 5);

	/* flush FIFO buffers */
	nrf_flush_tx(nrf);
	nrf_flush_rx(nrf);

	/* clear interrupt flags */
	magic =
	    BIT_STATUS_RX_DR | BIT_STATUS_TX_DS | BIT_STATUS_MAX_RT |
	    BIT_STATUS_TX_FULL;
	err = nrf_write_register(spi, REG_STATUS, &magic, 1);
	
	dev_info(&nrf->spi->dev, "nRF configured with status = 0x%02X\n", err);
	
	return 0;

errdev:
	unregister_chrdev(nrf_static_dev.nrf_major, DEVICE_NAME);

errchr:
	class_destroy(nrf_static_dev.nrf_class);

errclass:
	/* error registering class */

errirq:
	/* error requesting irq */
	free_irq(nrf->spi->irq, nrf_static_dev.dev);

errmem:
	kfree(nrf);
	nrf_static_dev.dev = nrf;
	
	return err;
}

/* un-configure IRQ, memory, SPI, character device */
static int __devexit nrf_remove(struct spi_device *spi)
{
	struct nrf24l01 *nrf = dev_get_drvdata(&spi->dev);
	int err;
	
	err = nrf_status(spi);
	dev_info(&nrf->spi->dev, "nRF exit, status = 0x%02X\n", err);

	free_irq(nrf->spi->irq, nrf_static_dev.dev);

	device_destroy(nrf_static_dev.nrf_class,
		       MKDEV(nrf_static_dev.nrf_major, 0));
	unregister_chrdev(nrf_static_dev.nrf_major, DEVICE_NAME);
	class_destroy(nrf_static_dev.nrf_class);

	kfree(nrf);
	nrf_static_dev.dev = nrf;

	return 0;
}

#ifdef CONFIG_PM
static int nrf_suspend(struct spi_device *spi, pm_message_t message)
{
	struct nrf24l01 *nrf = dev_get_drvdata(&spi->dev);

	dev_dbg(&spi->dev, "suspend\n");

	spin_lock_irq(&nrf->lock);

	disable_irq(nrf->spi->irq);

	nrf_mode_power_down(nrf);
	nrf_configure_radio_get(nrf);
	nrf_configure_pipe_get(nrf, 0);
	nrf_configure_pipe_get(nrf, 1);
	nrf_configure_pipe_get(nrf, 2);
	nrf_configure_pipe_get(nrf, 3);
	nrf_configure_pipe_get(nrf, 4);
	nrf_configure_pipe_get(nrf, 5);

	spin_unlock_irq(&nrf->lock);

	return 0;
}

static int nrf_resume(struct spi_device *spi)
{
	struct nrf24l01 *nrf = dev_get_drvdata(&spi->dev);

	spin_lock_irq(&nrf->lock);

	dev_dbg(&spi->dev, "resume\n");

	nrf_configure_radio(nrf, &nrf->radio);
	nrf_configure_pipe(nrf, &nrf->pipes[0]);
	nrf_configure_pipe(nrf, &nrf->pipes[1]);
	nrf_configure_pipe(nrf, &nrf->pipes[2]);
	nrf_configure_pipe(nrf, &nrf->pipes[3]);
	nrf_configure_pipe(nrf, &nrf->pipes[4]);
	nrf_configure_pipe(nrf, &nrf->pipes[5]);

	/* TX state is set to standby state as it will be invalid after suspend. */
	switch (nrf->state) {
	case STATE_POWER_DN:
		nrf_mode_power_down(nrf);
		break;

	case STATE_RX:
		nrf_mode_rx(nrf);
		break;

	case STATE_TX:
	case STATE_STANDBY:
	default:
		nrf_mode_standby(nrf);
		break;
	}

	enable_irq(nrf->spi->irq);

	spin_unlock_irq(&nrf->lock);

	return 0;
}
#else
#define nrf_suspend NULL
#define nrf_resume NULL
#endif

static struct spi_driver nrf_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = nrf_probe,
	.remove = nrf_remove,
	.suspend = nrf_suspend,
	.resume = nrf_resume,
};


/*
 * Module functions (init, exit)
 */

static int __init nrf_init_module(void)
{
	return spi_register_driver(&nrf_driver);
}

static void __exit nrf_cleanup_module(void)
{
	spi_unregister_driver(&nrf_driver);
}

module_init(nrf_init_module);
module_exit(nrf_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthew Dombroski");
MODULE_DESCRIPTION("Nordic Semiconductor nRF24L01 driver");
