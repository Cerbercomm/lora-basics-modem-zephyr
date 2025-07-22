#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include "sx126x_hal.h"
#include "sx126x_context.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sx126x_hal, CONFIG_LORA_BASICS_MODEM_LOG_LEVEL);

/* Definitions
*/

/* Internal
 */
static void sx126x_hal_wait_on_busy(const void* context) {
	const struct device *dev = (const struct device *)context;
	const struct sx126x_hal_config_t *config = dev->config;
	struct sx126x_hal_data_t *data = dev->data;

	// Implementation using GPIO Interrupt: wait to be signaled that BUSY dropped to 0
	if (gpio_pin_get_dt(&config->busy) != 0) {
		LOG_DBG("Waiting for BUSY");
		int ret = k_sem_take(&data->busy_sem, K_FOREVER);

		if (ret) {
			LOG_ERR("sx126x_hal_wait_on_busy():k_sem_take() returned: %d", ret);
		} else {
			LOG_DBG("Not BUSY anymore");
		}
	}
	
	/*
	// Implementation using polling
	bool success = WAIT_FOR(gpio_pin_get_dt(&config->busy) == 0,
		5000,
		k_usleep(100)
	);

	if (!success) {
		LOG_ERR("sx126x_hal_wait_on_busy() timed out");
	}*/
}

static void sx126x_hal_check_device_ready(const void* context) {
	const struct device *dev = (const struct device *)context;
	const struct sx126x_hal_config_t *config = dev->config;
	struct sx126x_hal_data_t *data = dev->data;

	if (data->radio_status != RADIO_SLEEP ) {
		sx126x_hal_wait_on_busy(context);
	} else {
		// Busy is HIGH in sleep mode, wake-up the device
		const struct gpio_dt_spec *cs = &config->spi.config.cs.gpio;

		gpio_pin_set_dt(cs, 1);
		sx126x_hal_wait_on_busy(context);
		gpio_pin_set_dt(cs, 0);
		data->radio_status = RADIO_AWAKE;
	}
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length, const uint8_t* data, const uint16_t data_length) {
	const struct device *dev = (const struct device *)context;
	const struct sx126x_hal_config_t *config = dev->config;
	struct sx126x_hal_data_t *dev_data = dev->data;
	int ret;

	sx126x_hal_check_device_ready(context);

	const struct spi_buf tx_bufs[] = {
		{
			.buf = (void *)command,
			.len = command_length
		}, {
			.buf = (void *)data,
			.len = data_length
		},
	};

	const struct spi_buf_set tx_buf_set = {tx_bufs, .count = ARRAY_SIZE(tx_bufs)};

	ret = spi_write_dt(&config->spi, &tx_buf_set);
	if (ret) {
		LOG_ERR("SPI write returned: %d", ret);
		return SX126X_HAL_STATUS_ERROR;
	}

	// 0x84 - SX126x_SET_SLEEP opcode. In sleep mode the radio dio is struck to 1
	// => do not test it
	if (command[0] == 0x84) {
		dev_data->radio_status = RADIO_SLEEP;
		//k_usleep(500);
	} else {
		sx126x_hal_check_device_ready(context);
	}

	/*LOG_HEXDUMP_INF(command, command_length, ">Write command");
	if (data_length > 0) LOG_HEXDUMP_INF(data, data_length, ">Write data");
	k_sleep(K_SECONDS(1));*/

	return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length, uint8_t* data, const uint16_t data_length) {
	const struct device *dev = (const struct device *)context;
	const struct sx126x_hal_config_t *config = dev->config;
	int ret;

	sx126x_hal_check_device_ready(context);

	const struct spi_buf tx_bufs[] = {
		{
			.buf = (uint8_t *)command,
			.len = command_length
		}, {
			.buf = NULL,
			.len = data_length
		}
	};

	const struct spi_buf rx_bufs[] = {
		{
			.buf = NULL,
			.len = command_length
		}, {
			.buf = data,
			.len = data_length
		}
	};

	const struct spi_buf_set tx_buf_set = {.buffers=tx_bufs, .count = ARRAY_SIZE(tx_bufs)};
	const struct spi_buf_set rx_buf_set = {.buffers=rx_bufs, .count = ARRAY_SIZE(rx_bufs)};

	ret = spi_transceive_dt(&config->spi, &tx_buf_set, &rx_buf_set);
	if (ret) {
		LOG_ERR("SPI transceive returned: %d", ret);
		return SX126X_HAL_STATUS_ERROR;
	}

	/*LOG_HEXDUMP_INF(command, command_length, "<Write command");
	LOG_HEXDUMP_INF(data, data_length, "<Read data");
	k_sleep(K_SECONDS(1));*/

	return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
	const struct device *dev = (const struct device *)context;
	const struct sx126x_hal_config_t *config = dev->config;
	struct sx126x_hal_data_t *data = dev->data;

	const struct gpio_dt_spec *nrst = &config->reset;

	gpio_pin_set_dt(nrst, 1);
	k_msleep(5);
	gpio_pin_set_dt(nrst, 0);
	k_msleep(5);

	data->radio_status = RADIO_AWAKE;
	return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
	sx126x_hal_check_device_ready(context);
	return SX126X_HAL_STATUS_OK;
}