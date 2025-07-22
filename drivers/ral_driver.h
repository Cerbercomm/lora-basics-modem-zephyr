#ifndef RAL_DRIVER_H
#define RAL_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include "lbm_p2p.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- Internal API implementation (to be used in DEVICE_API() *only*) ------------------------------------------------------------
 */
int ral_driver_init(const struct device *dev);

int ral_driver_set_frequency(const struct device *dev, uint32_t freq_in_hz);
int ral_driver_set_mod_param(const struct device *dev,
	enum lora_spread_factor sf, enum lora_signal_bandwidth bw, enum lora_coding_rate cr);
int ral_driver_set_pkt_param(const struct device *dev,
	uint16_t preamble_len_in_symb, enum lora_header_type ht, bool use_crc, bool invert_iq);
int ral_driver_set_tx_power(const struct device *dev, int8_t tx_power_in_dbm, uint32_t freq_in_hz);
int ral_driver_set_sync_word(const struct device *dev, uint8_t sync_word);
int ral_driver_sleep(const struct device *dev);
int ral_driver_send(const struct device *dev, uint8_t *data, uint32_t data_len);
int ral_driver_recv(const struct device *dev, uint8_t *data, uint8_t size, k_timeout_t timeout, int16_t *rssi, int8_t *snr);

#ifdef __cplusplus
}
#endif

#endif  // RAL_DRIVER_H
