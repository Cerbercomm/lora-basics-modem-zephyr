#include "ral_driver_legacy.h"
#include "ral.h"
#include "ral_defs.h"
#include "ral_driver_context.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LoRaP2P, CONFIG_LORA_BASICS_MODEM_LOG_LEVEL);

/* Definitions
*/
#define PRIVATE_SYNC_WORD 0x12
#define PUBLIC_SYNC_WORD  0x34

#define MAX_PACKET_SIZE_BYTES 255

struct lora_recv_callback_data_t {
    uint8_t *buffer;
    uint8_t size;
    int16_t rssi;
    int8_t snr;
};

// reference to device (we'll need this in the IRQ handler)
static const struct device *my_dev;

// packet parameters (used in configuration and when sending)
static ral_lora_pkt_params_t pkt_params;

// signal for Tx
struct k_poll_signal my_tx_signal;
static struct k_poll_signal *tx_signal;

// data for Rx
K_SEM_DEFINE(rx_sem, 0, 1);
static uint8_t pkt_payload[MAX_PACKET_SIZE_BYTES];
static lora_recv_cb rx_cb;
static void *rx_user_data;
static struct lora_recv_callback_data_t lora_recv_callback_data;

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

static ral_lora_sf_t convert_lora_sf(const enum lora_datarate datarate) {
    switch (datarate) {
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

        // signal Tx is done
        k_poll_signal_raise(tx_signal, 0x0);
    }

    // Rx Event ?
    if (irq & RAL_IRQ_RX_DONE) {
        LOG_DBG("Rx done");

        // get RSSI and SNR
        ral_lora_rx_pkt_status_t rx_pkt_status;
        status = ral_get_lora_rx_pkt_status(&hal_config->modem_radio.ral, &rx_pkt_status);
        if (status != RAL_STATUS_OK) {
            LOG_ERR("ral_get_lora_rx_pkt_status() returned: %d", status);
            return;
        }

        // get packet
        uint16_t size_in_bytes;
        status = ral_get_pkt_payload(&hal_config->modem_radio.ral,
            lora_recv_callback_data.size,
            lora_recv_callback_data.buffer,
            &size_in_bytes);

        if (status != RAL_STATUS_OK) {
            LOG_ERR("ral_get_pkt_payload() returned: %d", status);
            return;
        }

        // call callback with the data we gathered
        rx_cb(my_dev, lora_recv_callback_data.buffer,
            size_in_bytes, rx_pkt_status.rssi_pkt_in_dbm, rx_pkt_status.snr_pkt_in_db, rx_user_data);
    }
}

/* Rx callback
*/
static void lora_recv_callback(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr, void *user_data) {
    struct lora_recv_callback_data_t *lora_recv_callback_data = user_data;

    lora_recv_callback_data->rssi = rssi;
    lora_recv_callback_data->snr = snr;
    lora_recv_callback_data->size = size;
    // * we already have the packet payload in the buffer field, no need to copy it

    // signal we've got what we came for
    k_sem_give(&rx_sem);
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

    return 0;
}

int ral_driver_config(const struct device *dev, struct lora_modem_config *config) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    // keep reference of device
    my_dev = dev;

    // set packet type to LoRa
    ral_pkt_type_t pkt_type = RAL_PKT_TYPE_LORA;
    status = ral_set_pkt_type(&hal_config->modem_radio.ral, pkt_type);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_pkt_type(LORA) returned: %d", status);
        return -EINVAL;
    }

    // configure frequency
    status = ral_set_rf_freq(&hal_config->modem_radio.ral, config->frequency);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_rf_freq(%uHz) returned: %d", config->frequency, status);
        return -EINVAL;
    }

    // setup modulation
    ral_lora_mod_params_t mod_params;
    mod_params.sf = convert_lora_sf(config->datarate);
    mod_params.bw = convert_lora_bw(config->bandwidth);
    mod_params.cr = convert_lora_cr(config->coding_rate);
    mod_params.ldro = false;
    status = ral_set_lora_mod_params(&hal_config->modem_radio.ral, &mod_params);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_lora_mod_params() returned: %d", status);
        return -EINVAL;
    }

    // setup packet parameters
    pkt_params.preamble_len_in_symb = config->preamble_len;
    pkt_params.header_type = RAL_LORA_PKT_EXPLICIT;
    pkt_params.crc_is_on = true;
    pkt_params.invert_iq_is_on = config->iq_inverted;
    status = ral_set_lora_pkt_params(&hal_config->modem_radio.ral, &pkt_params);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_lora_pkt_params() returned: %d", status);
        return -EINVAL;
    }

    // set tx power
    status = ral_set_tx_cfg(&hal_config->modem_radio.ral, config->tx_power, config->frequency);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_tx_cfg(%ddBm) returned: %d", config->tx_power, status);
        return -EINVAL;
    }

    // set sync word
    int8_t sync_word = config->public_network ? PUBLIC_SYNC_WORD : PRIVATE_SYNC_WORD;
    status = ral_set_lora_sync_word(&hal_config->modem_radio.ral, sync_word);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_lora_sync_word(0x%02x) returned: %d", sync_word, status);
        return -EINVAL;
    }

    // we only care about successful Tx and Rx
    ral_irq_t irq = RAL_IRQ_ALL;//RAL_IRQ_TX_DONE | RAL_IRQ_RX_DONE;
    status = ral_set_dio_irq_params(&hal_config->modem_radio.ral, irq);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_dio_irq_params() returned: %d", status);
        return -EINVAL;
    }

    LOG_INF("Configuration done");

    return 0;
}

int ral_driver_send(const struct device *dev, uint8_t *data, uint32_t data_len) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    LOG_DBG("Sending %d bytes ..", data_len);

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

    // prepare Tx signal
    k_poll_signal_init(&my_tx_signal);
    tx_signal = &my_tx_signal;

    // send it on its way
    status = ral_set_tx(&hal_config->modem_radio.ral);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_tx() returned: %d", status);
        return -EINVAL;
    }

    // block until we know Tx is done
    struct k_poll_event events[1] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                 K_POLL_MODE_NOTIFY_ONLY,
                                 tx_signal),
    };

    // API doesnt pass a timeout value, so we wait forever for Tx to finish
    LOG_DBG("Waiting for Tx done ..");
    k_poll(events, 1, K_FOREVER);

    int signaled, result;
    k_poll_signal_check(tx_signal, &signaled, &result);

    if (signaled && (result == 0x0)) {
        // A-OK!
        return 0;
    } else {
        // weird error
        return -EIO;
    }
}

int ral_driver_send_async(const struct device *dev, uint8_t *data, uint32_t data_len, struct k_poll_signal *async) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    LOG_DBG("Sending async ..");

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

    // prepare Tx signal
    tx_signal = async;

    // send it on its way
    status = ral_set_tx(&hal_config->modem_radio.ral);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_tx() returned: %d", status);
        return -EINVAL;
    }

    return 0;
}

int ral_driver_recv(const struct device *dev, uint8_t *data, uint8_t size, k_timeout_t timeout, int16_t *rssi, int8_t *snr) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;
    int ret;

    LOG_DBG("Receiving ..");

    // assign callback
    lora_recv_callback_data.size = size;
    lora_recv_callback_data.buffer = data;
    rx_cb = lora_recv_callback;
    rx_user_data = &lora_recv_callback_data;

    // start listening (forever)
    status = ral_set_rx(&hal_config->modem_radio.ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_rx() returned: %d", status);
        return -EINVAL;
    }

    // wait on semaphore
    LOG_DBG("Waiting for Rx done ..");
    ret = k_sem_take(&rx_sem, timeout);
    if (ret != 0) {
        LOG_WRN("ral_driver_recv() timed out: %d", ret);
        return ret;
    } else {
        *rssi = lora_recv_callback_data.rssi;
        *snr = lora_recv_callback_data.snr;

        // return number of bytes received
        return lora_recv_callback_data.size;
    }
}

int ral_driver_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data) {
    struct ralf_hal_config_t *hal_config = dev->data;
    ral_status_t status;

    LOG_DBG("Receiving async ..");

    // assign callback
    lora_recv_callback_data.size = MAX_PACKET_SIZE_BYTES;
    lora_recv_callback_data.buffer = pkt_payload;
    rx_cb = cb;
    rx_user_data = user_data;

    // start listening (forever)
    status = ral_set_rx(&hal_config->modem_radio.ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
    if (status != RAL_STATUS_OK) {
        LOG_ERR("ral_set_rx() returned: %d", status);
        return -EINVAL;
    }

    return 0;
}
