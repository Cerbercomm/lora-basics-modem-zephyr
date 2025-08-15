#include "ral_defs.h"
#include "ral_driver.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SX126x, CONFIG_LORA_BASICS_MODEM_DRIVER_LOG_LEVEL);

#include "sx126x_context.h"
#include "ralf_sx126x.h"
#include "ralf.h"

// make sure we dont have collisions in the device tree
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(semtech_sx1261) +
        DT_NUM_INST_STATUS_OKAY(semtech_sx1262) +
	    DT_NUM_INST_STATUS_OKAY(semtech_sx1268) <= 1,
	    "Multiple SX126x instances in DT");

static struct sx126x_hal_data_t sx126x_data_lora;

/* BUSY handling
*/
static void sx126x_driver_busy_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	struct sx126x_hal_data_t *data = CONTAINER_OF(cb, struct sx126x_hal_data_t, busy_cb);

    // give semaphore (signal we're not busy anymore)
    k_sem_give(&data->busy_sem);

    LOG_DBG("sx126x_driver_busy_callback() finished");
}

/* DIO1 (sx126x interrupt) handling
*/
static void sx126x_driver_dio1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	//struct sx126x_hal_data_t *data = CONTAINER_OF(cb, struct sx126x_hal_data_t, busy_cb);

    // we've got work to do
    k_work_submit(&sx126x_data_lora.ralf.work);

    LOG_DBG("Submitted work");
}

/* Driver
*/
static int sx126x_init(const struct device *dev) {
	const struct sx126x_hal_config_t *config = dev->config;
	struct sx126x_hal_data_t *data = dev->data;
	int ret;

    /* Immutable configuration
    */
	if (!device_is_ready(config->spi.bus)) {
		LOG_ERR("SPI device not ready");
		return -EINVAL;
	}

	// Reset pin
	ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio_pin_configure_dt(RESET) returned: %d", ret);
		return ret;
	}

	// Busy pin
	ret = gpio_pin_configure_dt(&config->busy, GPIO_INPUT);
	if (ret < 0) {
        LOG_ERR("gpio_pin_configure_dt(BUSY) returned: %d", ret);
		return ret;
	}
    gpio_init_callback(&data->busy_cb, sx126x_driver_busy_callback, BIT(config->busy.pin));
    ret = gpio_add_callback(config->busy.port, &data->busy_cb);
    if (ret < 0) {
        LOG_ERR("gpio_add_callback(BUSY) returned: %d", ret);
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&config->busy, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret < 0) {
        LOG_ERR("gpio_pin_interrupt_configure_dt(BUSY) returned: %d", ret);
		return ret;
	}

	// DIO1 pin
    ret = gpio_pin_configure_dt(&config->dio1, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("gpio_pin_configure_dt(DIO1) returned: %d", ret);
        return ret;
    }
    gpio_init_callback(&data->dio1_cb, sx126x_driver_dio1_callback, BIT(config->dio1.pin));
    ret = gpio_add_callback(config->dio1.port, &data->dio1_cb);
    if (ret < 0) {
        LOG_ERR("gpio_add_callback(DIO1) returned: %d", ret);
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&config->dio1, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("gpio_pin_interrupt_configure_dt(DIO1) returned: %d", ret);
		return ret;
	}

    // RX Enable pin
    if (config->rx_enable.port) {
        ret = gpio_pin_configure_dt(&config->rx_enable, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("gpio_pin_configure_dt(RX ENABLE) returned: %d", ret);
			return ret;
		}
    }

    /* Mutable data
    */
    // radio status
	data->radio_status = RADIO_AWAKE;

    // semaphore for signaling BUSY state
    ret = k_sem_init(&data->busy_sem, 0, 1);
    if (ret < 0) {
        LOG_ERR("k_sem_init(BUSY) returned: %d", ret);
		return ret;
	}

    LOG_DBG("Lora env initialized");

    /* set Radio Abstration Layer deFinition driver to SX126X
    */
    ralf_t modem_radio = RALF_SX126X_INSTANTIATE(dev);
    data->ralf.modem_radio.ral = modem_radio.ral;
    data->ralf.modem_radio.ralf_drv = modem_radio.ralf_drv;

    // initialize the hardware
    ret = ral_driver_init(dev);
    if (ret < 0) {
        LOG_ERR("ral_driver_init() returned: %d", ret);
		return ret;
	}
    
    LOG_DBG("Lora ready");

	return ret;
}

/* Driver API
*/
static uint32_t sx126x_driver_get_mtu(const struct device *dev) {
	return 255;
}

static int sx126x_driver_send(const struct device *dev, uint8_t *data, uint32_t data_len) {
    const struct sx126x_hal_config_t *config = dev->config;
    int ret;

    // enable sending (if needed)
    if (config->rx_enable.port) {
        ret = gpio_pin_set_dt(&config->rx_enable, 0);
        if (ret < 0) {
			LOG_ERR("sx126x_driver_send():gpio_pin_set_dt(RX ENABLE) returned: %d", ret);
			return ret;
		}
    }

    return ral_driver_send(dev, data, data_len);
}

static int sx126x_driver_recv(const struct device *dev, uint8_t *data, uint8_t size, k_timeout_t timeout, int16_t *rssi, int8_t *snr) {
    const struct sx126x_hal_config_t *config = dev->config;
    int ret;

    // enable receiving (if needed)
    if (config->rx_enable.port) {
        ret = gpio_pin_set_dt(&config->rx_enable, 1);
        if (ret < 0) {
			LOG_ERR("sx126x_driver_recv():gpio_pin_set_dt(RX ENABLE) returned: %d", ret);
			return ret;
		}
    }

    return ral_driver_recv(dev, data, size, timeout, rssi, snr);
}

/* Device definitions
*/
#define SX126X_SPI_OPERATION (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB)

#define CONFIGURE_GPIO_IF_IN_DT(node_id, name, dt_prop)                       \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, dt_prop),                           \
		(.name = GPIO_DT_SPEC_GET(node_id, dt_prop),),                        \
		())

static const struct sx126x_hal_config_t sx126x_config_lora = {
    .spi = SPI_DT_SPEC_GET(DT_NODELABEL(lora), SX126X_SPI_OPERATION, 0),

    .reset = GPIO_DT_SPEC_GET(DT_NODELABEL(lora), reset_gpios),
    .busy = GPIO_DT_SPEC_GET(DT_NODELABEL(lora), busy_gpios),
    .dio1 = GPIO_DT_SPEC_GET(DT_NODELABEL(lora), dio1_gpios),

    // the RF Switch control pin has multiple possible names, but all have same functionality
    CONFIGURE_GPIO_IF_IN_DT(DT_NODELABEL(lora), rx_enable, rx_enable_gpios)
    CONFIGURE_GPIO_IF_IN_DT(DT_NODELABEL(lora), rx_enable, antenna_enable_gpios)

    .dio2_as_rf_switch = DT_PROP(DT_NODELABEL(lora), dio2_tx_enable),

    .xosc_cfg = COND_CODE_1(DT_PROP(DT_NODELABEL(lora), tcxo_power_startup_delay_ms) == 0,
                                (RAL_XOSC_CFG_XTAL),
                                (COND_CODE_1(DT_PROP(DT_NODELABEL(lora), dio3_tcxo_voltage) == 0,
                                    (RAL_XOSC_CFG_TCXO_EXT_CTRL),
                                    (RAL_XOSC_CFG_TCXO_RADIO_CTRL))
                                )
                            ),
    .tcxo_voltage = DT_PROP_OR(DT_NODELABEL(lora), dio3_tcxo_voltage, 0xFF),
    .tcxo_wakeup_time_ms = DT_PROP_OR(DT_NODELABEL(lora), tcxo_power_startup_delay_ms, 0xFF),

    .capa_xta = DT_PROP_OR(DT_NODELABEL(lora), xtal_capacitor_value_xta, 0xFF),
    .capa_xtb = DT_PROP_OR(DT_NODELABEL(lora), xtal_capacitor_value_xtb, 0xFF),

    .reg_mode = SX126X_REG_MODE_DCDC,

    .rx_boosted = ((CONFIG_LORA_BASICS_MODEM_BOOST_RX) == 1)
};

static DEVICE_API(lbm, sx126x_lora_api) = {
    .get_mtu       = sx126x_driver_get_mtu,
    .set_frequency = ral_driver_set_frequency,
    .set_mod_param = ral_driver_set_mod_param,
    .set_pkt_param = ral_driver_set_pkt_param,
    .set_tx_power  = ral_driver_set_tx_power,
    .set_sync_word = ral_driver_set_sync_word,
    .sleep         = ral_driver_sleep,
    .recv          = sx126x_driver_recv,
    .send          = sx126x_driver_send
};

DEVICE_DT_DEFINE(DT_NODELABEL(lora), sx126x_init, NULL,
    &sx126x_data_lora, &sx126x_config_lora,
    POST_KERNEL, CONFIG_LORA_BASICS_MODEM_INIT_PRIORITY, &sx126x_lora_api);