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

	struct nrf_payload *payload_user = (struct nrf_payload *) arg;
	struct nrf_reg *reg_user = (struct nrf_reg *) arg;

	char* buf;
	int tmp;

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

	case NRF_IOC_R_REG:
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
		
	case NRF_IOC_W_REG:
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
		
	case NRF_IOC_RX_PAYLOAD:
		/* allocate maximum size buffer */
		buf = kzalloc(sizeof(char) * MAX_TX_BYTES, GFP_KERNEL);
		if (buf == NULL) {
			return -EFAULT;
		}

		err = nrf_read_payload(nrf->spi, buf, payload_user->len);
		if( err ) {
			kfree(buf);
			return err;
		}

		/* copy the payload to the user buffer */
		err = copy_to_user(payload_user->data, buf, payload_user->len);
		if (err) {
			kfree(buf);
			return -EFAULT;
		}
		
		kfree(buf);
		break;
		
	case NRF_IOC_TX_PAYLOAD:
		buf = kzalloc(sizeof(char) * payload_user->len, GFP_KERNEL);
		if (buf == NULL)
			return -EFAULT;
		
		if (copy_from_user(buf, payload_user->data, payload_user->len)) {
			kfree(buf);
			return -EFAULT;
		}

		err = nrf_write_payload(nrf->spi, buf, payload_user->len);

		kfree(buf);
		break;
		
	case NRF_IOC_ACK_PAYLOAD:
		buf = kzalloc(sizeof(char) * payload_user->len, GFP_KERNEL);
		if (buf == NULL)
			return -EFAULT;
		
		if (copy_from_user(buf, payload_user->data, payload_user->len)) {
			kfree(buf);
			return -EFAULT;
		}
		
		err = nrf_write_ack_payload(nrf->spi, payload_user->pipe, buf, payload_user->len);
		
		kfree(buf);
		break;
		
	case NRF_IOC_NOACK_PAYLOAD:
		buf = kzalloc(sizeof(char) * payload_user->len, GFP_KERNEL);
		if (buf == NULL)
			return -EFAULT;
		
		if (copy_from_user(buf, payload_user->data, payload_user->len)) {
			kfree(buf);
			return -EFAULT;
		}
		
		err = nrf_write_payload_noack(nrf->spi, buf, payload_user->len);
		
		kfree(buf);
		break;
		
	case NRF_IOC_R_RX_LENGTH:
		tmp = nrf_read_payload_width(nrf->spi);
		copy_to_user(arg, &tmp, sizeof(tmp));
		break;
		
	case NRF_IOC_REUSE_PAYLOAD:
		dev_err(&nrf->spi->dev, "This ioctl isnt implemented yet\n");
		return -EINVAL;
		break;

	case NRF_IOC_FLUSH_RX:
		nrf_flush_rx(nrf);
		break;
		
	case NRF_IOC_FLUSH_TX:
		nrf_flush_tx(nrf);
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
		
	case NRF_IOC_SERVICE_IRQ:
		tmp = nrf_service_irq(nrf);
		copy_to_user(arg, &tmp, sizeof(tmp));
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

	spin_unlock_irq(&nrf->lock);

	return 0;
}

static int nrf_resume(struct spi_device *spi)
{
	struct nrf24l01 *nrf = dev_get_drvdata(&spi->dev);

	spin_lock_irq(&nrf->lock);

	dev_dbg(&spi->dev, "resume\n");

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
