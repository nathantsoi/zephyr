/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <i2c.h>
#include <clock_control.h>
#include <fsl_lpi2c.h>
#include <misc/util.h>
#include <soc.h>
#include "i2c-priv.h"

#define DEV_CFG(dev) \
	((const struct i2c_mcux_lpi2c_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2c_mcux_lpi2c_data * const)(dev)->driver_data)
#define DEV_BASE(dev) \
	((LPI2C_Type *)(DEV_CFG(dev))->base)

struct i2c_mcux_lpi2c_config {
	LPI2C_Type *base;
	char *clock_name;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(struct device *dev);
	u32_t bitrate;
};

struct i2c_mcux_lpi2c_data {
	lpi2c_master_handle_t handle;
	struct k_sem device_sync_sem;
	status_t callback_status;
};

static int i2c_mcux_lpi2c_configure(struct device *dev, u32_t dev_config_raw)
{
	LPI2C_Type *base = DEV_BASE(dev);
	const struct i2c_mcux_lpi2c_config *config = DEV_CFG(dev);
	struct device *clock_dev;
	u32_t clock_freq;
	u32_t baudrate;

	if (!(I2C_MODE_MASTER & dev_config_raw)) {
		return -EINVAL;
	}

	if (I2C_ADDR_10_BITS & dev_config_raw) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		baudrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		baudrate = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	clock_dev = device_get_binding(config->clock_name);
	if (clock_dev == NULL) {
		return -EINVAL;
	}

	if (clock_control_get_rate(clock_dev, config->clock_subsys, &clock_freq)) {
		return -EINVAL;
	}

	LPI2C_MasterSetBaudRate(base, clock_freq, baudrate);

	return 0;
}

static void i2c_mcux_lpi2c_master_transfer_callback(LPI2C_Type *base,
		lpi2c_master_handle_t *handle, status_t status, void *userData)
{
	struct device *dev = userData;
	struct i2c_mcux_lpi2c_data *data = DEV_DATA(dev);

	ARG_UNUSED(handle);
	ARG_UNUSED(base);

	data->callback_status = status;
	k_sem_give(&data->device_sync_sem);
}

static u32_t i2c_mcux_lpi2c_convert_flags(int msg_flags)
{
	u32_t flags = 0;

	if (!(msg_flags & I2C_MSG_STOP)) {
		flags |= kLPI2C_TransferNoStopFlag;
	}

	if (msg_flags & I2C_MSG_RESTART) {
		flags |= kLPI2C_TransferRepeatedStartFlag;
	}

	return flags;
}

static int i2c_mcux_lpi2c_transfer(struct device *dev, struct i2c_msg *msgs,
		u8_t num_msgs, u16_t addr)
{
	LPI2C_Type *base = DEV_BASE(dev);
	struct i2c_mcux_lpi2c_data *data = DEV_DATA(dev);
	lpi2c_master_transfer_t transfer;
	status_t status;

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {

		/* Initialize the transfer descriptor */
		transfer.flags = i2c_mcux_lpi2c_convert_flags(msgs->flags);
		transfer.slaveAddress = addr;
		transfer.direction = (msgs->flags & I2C_MSG_READ)
			? kLPI2C_Read : kLPI2C_Write;
		transfer.subaddress = 0;
		transfer.subaddressSize = 0;
		transfer.data = msgs->buf;
		transfer.dataSize = msgs->len;

		/* Start the transfer */
		status = LPI2C_MasterTransferNonBlocking(base, &data->handle, &transfer);

		/* Return an error if the transfer didn't start successfully
		 * e.g., if the bus was busy
		 */
		if (status == kStatus_Fail) {
			return -EIO;
		}

		/* Wait for the transfer to complete */
		k_sem_take(&data->device_sync_sem, K_FOREVER);

		/* Return an error if the transfer didn't complete
		 * successfully. e.g., nak, timeout, lost arbitration
		 */
		if (data->callback_status != kStatus_Success) {
			return -EIO;
		}

		/* Move to the next message */
		msgs++;
	}

	return 0;
}

static void i2c_mcux_lpi2c_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	LPI2C_Type *base = DEV_BASE(dev);
	struct i2c_mcux_lpi2c_data *data = DEV_DATA(dev);

	LPI2C_MasterTransferHandleIRQ(base, &data->handle);
}

static int i2c_mcux_lpi2c_init(struct device *dev)
{
	LPI2C_Type *base = DEV_BASE(dev);
	const struct i2c_mcux_lpi2c_config *config = DEV_CFG(dev);
	struct i2c_mcux_lpi2c_data *data = DEV_DATA(dev);
	u32_t clock_freq, bitrate_cfg;
	lpi2c_master_config_t master_config;
	struct device *clock_dev;
	int error;

	k_sem_init(&data->device_sync_sem, 0, UINT_MAX);

	clock_dev = device_get_binding(config->clock_name);
	if (clock_dev == NULL) {
		return -EINVAL;
	}

	if (clock_control_get_rate(clock_dev, config->clock_subsys, &clock_freq)) {
		return -EINVAL;
	}

	LPI2C_MasterGetDefaultConfig(&master_config);
	LPI2C_MasterInit(base, &master_config, clock_freq);
	LPI2C_MasterTransferCreateHandle(base, &data->handle,
			i2c_mcux_lpi2c_master_transfer_callback, dev);

	bitrate_cfg = _i2c_map_dt_bitrate(config->bitrate);

	error = i2c_mcux_lpi2c_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
	if (error) {
		return error;
	}

	config->irq_config_func(dev);

	return 0;
}

static const struct i2c_driver_api i2c_mcux_lpi2c_driver_api = {
	.configure = i2c_mcux_lpi2c_configure,
	.transfer = i2c_mcux_lpi2c_transfer,
};

#ifdef CONFIG_I2C_MCUX_LPI2C_1
static void i2c_mcux_lpi2c_config_func_1(struct device *dev);

static const struct i2c_mcux_lpi2c_config i2c_mcux_lpi2c_config_1 = {
	.base = (LPI2C_Type *)CONFIG_I2C_MCUX_LPI2C_1_BASE_ADDRESS,
	.clock_name = CONFIG_I2C_MCUX_LPI2C_1_CLOCK_NAME,
	.clock_subsys =
		(clock_control_subsys_t)CONFIG_I2C_MCUX_LPI2C_1_CLOCK_SUBSYS,
	.irq_config_func = i2c_mcux_lpi2c_config_func_1,
	.bitrate = CONFIG_I2C_MCUX_LPI2C_1_BITRATE,
};

static struct i2c_mcux_lpi2c_data i2c_mcux_lpi2c_data_1;


DEVICE_AND_API_INIT(i2c_mcux_lpi2c_1, CONFIG_I2C_MCUX_LPI2C_1_NAME, &i2c_mcux_lpi2c_init,
		    &i2c_mcux_lpi2c_data_1, &i2c_mcux_lpi2c_config_1,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &i2c_mcux_lpi2c_driver_api);

static void i2c_mcux_lpi2c_config_func_1(struct device *dev)
{
	IRQ_CONNECT(CONFIG_I2C_MCUX_LPI2C_1_IRQ, CONFIG_I2C_MCUX_LPI2C_1_IRQ_PRI,
		    i2c_mcux_lpi2c_isr, DEVICE_GET(i2c_mcux_lpi2c_1), 0);

	irq_enable(CONFIG_I2C_MCUX_LPI2C_1_IRQ);
}
#endif /* CONFIG_I2C_MCUX_LPI2C_1 */

