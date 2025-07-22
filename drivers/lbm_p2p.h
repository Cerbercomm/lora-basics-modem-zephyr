#ifndef LBM_P2P_H
#define LBM_P2P_H

#include "zephyr/toolchain.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
/**
 * @brief LoRa sync word to be used in private networks
 */
#define PRIVATE_LORA_SYNC_WORD 0x12

/**
 * @brief LoRa sync word to be used in public networks
 */
#define PUBLIC_LORA_SYNC_WORD  0x34

/**
 * @brief LoRa signal bandwidth
 */
enum lora_signal_bandwidth {
	BW_125_KHZ = 0,
	BW_250_KHZ,
	BW_500_KHZ,
};

/**
 * @brief LoRa data-rate
 */
enum lora_spread_factor {
	SF_6 = 6,
	SF_7,
	SF_8,
	SF_9,
	SF_10,
	SF_11,
	SF_12,
};

/**
 * @brief LoRa coding rate
 */
enum lora_coding_rate {
	CR_4_5 = 1,
	CR_4_6 = 2,
	CR_4_7 = 3,
	CR_4_8 = 4,
};

/**
 * @brief LoRa header type
 */
enum lora_header_type {
	LORA_HEADER_IMPLICIT,
	LORA_HEADER_EXPLICIT
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal driver use only, skip these in public documentation.
*/
typedef int (*lbm_api_set_frequency)(const struct device *dev, uint32_t freq_in_hz);
typedef int (*lbm_api_set_mod_param)(const struct device *dev,
	enum lora_spread_factor sf, enum lora_signal_bandwidth bw, enum lora_coding_rate cr);
typedef int (*lbm_api_set_pkt_param)(const struct device *dev,
	uint16_t preamble_len_in_symb, enum lora_header_type ht, bool use_crc, bool invert_iq);
typedef int (*lbm_api_set_tx_power)(const struct device *dev, int8_t tx_power_in_dbm, uint32_t freq_in_hz);
typedef int (*lbm_api_set_sync_word)(const struct device *dev, uint8_t sync_word);
typedef int (*lbm_api_sleep)(const struct device *dev);

typedef uint32_t (*lbm_api_get_mtu)(const struct device *dev);
typedef int (*lbm_api_send)(const struct device *dev, uint8_t *data, uint32_t data_len);
typedef int (*lbm_api_recv)(const struct device *dev, uint8_t *data, uint8_t size, k_timeout_t timeout, int16_t *rssi, int8_t *snr);

__subsystem struct lbm_driver_api {
	lbm_api_set_frequency set_frequency;
	lbm_api_set_mod_param set_mod_param;
	lbm_api_set_pkt_param set_pkt_param;
	lbm_api_set_tx_power  set_tx_power;
	lbm_api_set_sync_word set_sync_word;
	lbm_api_sleep         sleep;
	lbm_api_get_mtu       get_mtu;
	lbm_api_send          send;
	lbm_api_recv          recv;
};

/** @endcond */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC DRIVER API -------------------------------------------------------
*/
/**
 * @brief Set the frequency
 *
 * @param dev        LBM (Lora Basics Modem) device
 * @param freq_in_hz Frequency in Hertz
 * @return 0 on success, negative on error
 */
static inline int lbm_set_frequency(const struct device *dev, const uint32_t freq_in_hz) {
	return DEVICE_API_GET(lbm, dev)->set_frequency(dev, freq_in_hz);
}

/**
 * @brief Set the modulation parameters
 *
 * @param dev LBM (Lora Basics Modem) device
 * @param sf  Spread factor
 * @param bw  Band width
 * @param cr  Coding rate
 * @return 0 on success, negative on error
 */
static inline int lbm_set_mod_param(const struct device *dev, enum lora_spread_factor sf, enum lora_signal_bandwidth bw, enum lora_coding_rate cr) {
	return DEVICE_API_GET(lbm, dev)->set_mod_param(dev, sf, bw, cr);
}

/**
 * @brief Set the packet parameters
 *
 * @param dev                  LBM (Lora Basics Modem) device
 * @param preamble_len_in_symb Preamble length in symbols
 * @param ht                   Header type (implicit or explicit)
 * @param use_crc              Use CRC checksum in header
 * @param invert_iq            Invert IQ channels
 * @return 0 on success, negative on error
 */
static inline int lbm_set_pkt_param(const struct device *dev, uint16_t preamble_len_in_symb, enum lora_header_type ht, bool use_crc, bool invert_iq) {
	return DEVICE_API_GET(lbm, dev)->set_pkt_param(dev, preamble_len_in_symb, ht, use_crc, invert_iq);
}

/**
 * @brief Set TX power (before transmitting)
 *
 * @param dev             LBM (Lora Basics Modem) device
 * @param tx_power_in_dbm Tx power in dBm
 * @param freq_in_hz      Frequency in Hertz
 * @return 0 on success, negative on error
 */
static inline int lbm_set_tx_power(const struct device *dev, int8_t tx_power_in_dbm, uint32_t freq_in_hz) {
	return DEVICE_API_GET(lbm, dev)->set_tx_power(dev, tx_power_in_dbm, freq_in_hz);
}

/**
 * @brief Set Sync word
 *
 * @note Usually 0x12 for private networkds and 0x34 for public networks
 *
 * @param dev       LBM (Lora Basics Modem) device
 * @param sync_word Sync word
 * @return 0 on success, negative on error
 */
static inline int lbm_set_sync_word(const struct device *dev, uint8_t sync_word) {
	return DEVICE_API_GET(lbm, dev)->set_sync_word(dev, sync_word);
}

/**
 * @brief Put Lora device to sleep
 *
 * @note Device will wake up by itself on next API call
 *
 * @param dev       LBM (Lora Basics Modem) device
 * @return 0 on success, negative on error
 */
static inline int lbm_sleep(const struct device *dev) {
	return DEVICE_API_GET(lbm, dev)->sleep(dev);
}

/**
 * @brief Get MTU (maximum transfer unit in bytes) supported by the hardware
 *
 * @param dev      LBM (Lora Basics Modem) device
 * @return MTU in bytes
 */
static inline uint32_t lbm_get_mtu(const struct device *dev) {
	return DEVICE_API_GET(lbm, dev)->get_mtu(dev);
}

/**
 * @brief Send data over LoRa
 *
 * @note This blocks until transmission is complete.
 *
 * @param dev      LBM (Lora Basics Modem) device
 * @param data     Data to be sent
 * @param data_len Length of the data to be sent
 * @return 0 on success, negative on error
 */
static inline int lbm_send(const struct device *dev, uint8_t *data, uint32_t data_len) {
	return DEVICE_API_GET(lbm, dev)->send(dev, data, data_len);
}

/**
 * @brief Receive data over LoRa
 *
 * @note This blocks until data received or a timeout occurs.
 *
 * @param dev       LBM (Lora Basics Modem) device
 * @param data      Buffer to hold received data
 * @param size      Size of the buffer to hold the received data. Max size allowed is 255.
 * @param timeout   Duration to wait for a packet.
 * @param rssi      RSSI of received data
 * @param snr       SNR of received data
 * @return Length of the data received on success, negative on error
 */
static inline int lbm_recv(const struct device *dev, uint8_t *data, uint8_t size, k_timeout_t timeout, int16_t *rssi, int8_t *snr) {
	return DEVICE_API_GET(lbm, dev)->recv(dev, data, size, timeout, rssi, snr);
}

#ifdef __cplusplus
}
#endif

#endif  // LBM_P2P_H
