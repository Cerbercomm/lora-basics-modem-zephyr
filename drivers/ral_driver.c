#include "ral_driver.h"
#include "ral.h"
#include "ral_defs.h"
#include "ral_driver_context.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LoRaP2P, CONFIG_LORA_BASICS_MODEM_LOG_LEVEL);

// packet parameters needs to persist to be used when sending
static ral_lora_pkt_params_t pkt_params;

// semaphore for Rx
static K_SEM_DEFINE(rx_sem, 0, 1);

// semaphore for Tx
static K_SEM_DEFINE(tx_sem, 0, 1);

/* Internal
*/
static ral_lora_bw_t convert_lora_bw(const enum lora_signal_bandwidth bandwidth) {
    switch (bandwidth) {
    case BW_125_KHZ:
        return RAL_LORA_BW_125_KHZ;
    default:
    case BW_250_KHZ:
        return RAL_LORA_BW_250_KHZ;
    case BW_500_KHZ:
        return RAL_LORA_BW_500_KHZ;
    }
}

static ral_lora_sf_t convert_lora_sf(const enum lora_spread_factor sf) {
    switch (sf) {
    case SF_6:
        return RAL_LORA_SF6;
    case SF_7:
        return RAL_LORA_SF7;
    case SF_8:
        return RAL_LORA_SF8;
    default:
    case SF_9:
        return RAL_LORA_SF9;
    case SF_10:
        return RAL_LORA_SF10;
    case SF_11:
        return RAL_LORA_SF11;
    case SF_12:
      return RAL_LORA_SF12;
    }
}

static ral_lora_cr_t convert_lora_cr(const enum lora_coding_rate cr) {
    switch (cr) {
    default:
    case CR_4_5:
        return RAL_LORA_CR_4_5;
    case CR_4_6:
        return RAL_LORA_CR_4_6;
    case CR_4_7:
        return RAL_LORA_CR_4_7;
    case CR_4_8:
        return RAL_LORA_CR_4_8;
    }
}

static ral_lora_pkt_len_modes_t convert_lora_header(const enum lora_header_type ht) {
    switch (ht) {
    case LORA_HEADER_IMPLICIT:
        return RAL_LORA_PKT_IMPLICIT;
    default:
    case LORA_HEADER_EXPLICIT:
        return RAL_LORA_PKT_EXPLICIT;
    }
}

/* LoRa IRQ handler
*/
void irq_event_handler(struct k_work *item) {
    ARG_UNUSED(item);

    struct ralf_hal_config_t *hal_config = CONTAINER_OF(item, struct ralf_hal_config_t, work);
    ral_status_t status;

    // figure what happened
    ral_irq_t irq;
    status = ral_get_and_clear_irq_status(&hal_config->modem_radio.ral, &irq);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_get_and_clear_irq_status() returned: %d", status);
        return;
    }

    // Tx Event ?
    if (irq & RAL_IRQ_TX_DONE) {
        LOG_DBG("Tx done");

        k_sem_give(&tx_sem);
    }

    // Rx Event ?
    else if (irq & RAL_IRQ_RX_DONE) {
        LOG_DBG("Rx done");

        k_sem_give(&rx_sem);
    }
}

/* Driver API
*/
int ral_driver_init(const struct device *dev) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    // initialize work handler
    k_work_init(&hal_config->work, irq_event_handler);

    // reset hardware
    status = ral_reset(&hal_config->modem_radio.ral);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_reset() returned: %d", status);
        return -EINVAL;
    }

    // initialize hardware
    status = ral_init(&hal_config->modem_radio.ral);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_init() returned: %d", status);
        return -EINVAL;
    }

    // put in standby mode
    ral_standby_cfg_t standby_cfg = RAL_STANDBY_CFG_RC;
    status = ral_set_standby(&hal_config->modem_radio.ral, standby_cfg);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_standby() returned: %d", status);
        return -EINVAL;
    }

    LOG_DBG("ral_driver_init() done");

    return 0;
}

int ral_driver_set_frequency(const struct device *dev, uint32_t freq_in_hz) {
    LOG_DBG("ral_driver_set_frequency() called");

    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    // configure frequency
    status = ral_set_rf_freq(&hal_config->modem_radio.ral, freq_in_hz);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_rf_freq(%uHz) returned: %d", freq_in_hz, status);
        return -EINVAL;
    }

    LOG_DBG("ral_driver_set_frequency() done");

    return 0;
}

int ral_driver_set_mod_param(const struct device *dev, enum lora_spread_factor sf, enum lora_signal_bandwidth bw, enum lora_coding_rate cr) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    // set packet type to LoRa
    ral_pkt_type_t pkt_type = RAL_PKT_TYPE_LORA;
    status = ral_set_pkt_type(&hal_config->modem_radio.ral, pkt_type);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_pkt_type(LORA) returned: %d", status);
        return -EINVAL;
    }

    // setup modulation
    ral_lora_mod_params_t mod_params;
    mod_params.sf = convert_lora_sf(sf);
    mod_params.bw = convert_lora_bw(bw);
    mod_params.cr = convert_lora_cr(cr);
    mod_params.ldro = false;
    status = ral_set_lora_mod_params(&hal_config->modem_radio.ral, &mod_params);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_lora_mod_params() returned: %d", status);
        return -EINVAL;
    }

    LOG_DBG("ral_driver_set_mod_param() done");

    return 0;
}

int ral_driver_set_pkt_param(const struct device *dev, uint16_t preamble_len_in_symb, enum lora_header_type ht, bool use_crc, bool invert_iq) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    // setup packet parameters
    pkt_params.preamble_len_in_symb = preamble_len_in_symb;
    pkt_params.header_type = convert_lora_header(ht);
    pkt_params.crc_is_on = use_crc;
    pkt_params.invert_iq_is_on = invert_iq;
    status = ral_set_lora_pkt_params(&hal_config->modem_radio.ral, &pkt_params);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_lora_pkt_params() returned: %d", status);
        return -EINVAL;
    }

    LOG_DBG("ral_driver_set_pkt_param() done");

    return 0;
}

int ral_driver_set_tx_power(const struct device *dev, int8_t tx_power_in_dbm, uint32_t freq_in_hz) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    // set tx power
    status = ral_set_tx_cfg(&hal_config->modem_radio.ral, tx_power_in_dbm, freq_in_hz);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_tx_cfg(%ddBm) returned: %d", tx_power_in_dbm, status);
        return -EINVAL;
    }

    LOG_DBG("ral_driver_set_tx_power() done");

    return 0;
}

int ral_driver_set_sync_word(const struct device *dev, uint8_t sync_word) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    status = ral_set_lora_sync_word(&hal_config->modem_radio.ral, sync_word);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_lora_sync_word(0x%02x) returned: %d", sync_word, status);
        return -EINVAL;
    }

    LOG_DBG("ral_driver_set_sync_word() done");

    return 0;
}

int ral_driver_sleep(const struct device *dev) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    status = ral_set_sleep(&hal_config->modem_radio.ral, true);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_sleep() returned: %d", status);
        return -EINVAL;
    }

    LOG_DBG("ral_driver_sleep() done");

    return 0;
}

int ral_driver_send(const struct device *dev, uint8_t *data, uint32_t data_len) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    // we're interested in successful send
    ral_irq_t irq = RAL_IRQ_TX_DONE;
    status = ral_set_dio_irq_params(&hal_config->modem_radio.ral, irq);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_dio_irq_params() returned: %d", status);
        return -EINVAL;
    }

    // set payload
    status = ral_set_pkt_payload(&hal_config->modem_radio.ral, data, data_len);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_pkt_payload() returned: %d", status);
        return -EINVAL;
    }

    // set packet size
    pkt_params.pld_len_in_bytes = data_len;
    status = ral_set_lora_pkt_params(&hal_config->modem_radio.ral, &pkt_params);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_lora_pkt_params() returned: %d", status);
        return -EINVAL;
    }

    // send it on its way
    status = ral_set_tx(&hal_config->modem_radio.ral);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_tx() returned: %d", status);
        return -EINVAL;
    }

    // wait on Tx semaphore
    LOG_DBG("Waiting for Tx done ..");
    int ret = k_sem_take(&tx_sem, K_FOREVER);
    if (ret != 0) {
        LOG_WRN("ral_driver_send() failed: %d", ret);
        return ret;

    }
    
    // not listening to IRQs anymore
    irq = RAL_IRQ_NONE;
    status = ral_set_dio_irq_params(&hal_config->modem_radio.ral, irq);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_dio_irq_params() returned: %d", status);
        return -EINVAL;
    }

    LOG_DBG("ral_driver_send() done");
    
    return 0;
}

int ral_driver_recv(const struct device *dev, uint8_t *data, uint8_t size, k_timeout_t timeout, int16_t *rssi, int8_t *snr) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;
    int ret;

    // we're interested in successful recv
    ral_irq_t irq = RAL_IRQ_RX_DONE;
    status = ral_set_dio_irq_params(&hal_config->modem_radio.ral, irq);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_dio_irq_params() returned: %d", status);
        return -EINVAL;
    }

    // start listening (forever)
    status = ral_set_rx(&hal_config->modem_radio.ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_rx() returned: %d", status);
        return -EINVAL;
    }

    // wait on Rx semaphore
    LOG_DBG("Waiting for Rx done ..");
    ret = k_sem_take(&rx_sem, timeout);
    if (ret != 0) {
        LOG_DBG("ral_driver_recv() timed out: %d", ret);
        return -ETIMEDOUT;
    }

    // not listening to IRQs anymore
    irq = RAL_IRQ_NONE;
    status = ral_set_dio_irq_params(&hal_config->modem_radio.ral, irq);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_dio_irq_params() returned: %d", status);
        return -EINVAL;
    }

    // get RSSI and SNR
    ral_lora_rx_pkt_status_t rx_pkt_status;
    status = ral_get_lora_rx_pkt_status(&hal_config->modem_radio.ral, &rx_pkt_status);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_get_lora_rx_pkt_status() returned: %d", status);
        return -EINVAL;
    }
    *rssi = rx_pkt_status.rssi_pkt_in_dbm;
    *snr = rx_pkt_status.snr_pkt_in_db;

    // get packet
    uint16_t size_in_bytes;
    status = ral_get_pkt_payload(&hal_config->modem_radio.ral,
        size,
        data,
        &size_in_bytes);

    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_get_pkt_payload() returned: %d", status);
        return -EINVAL;
    }

    return size_in_bytes;
}
