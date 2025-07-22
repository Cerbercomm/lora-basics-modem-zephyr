#ifndef RAL_DRIVER_LEGACY_H
#define RAL_DRIVER_LEGACY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/lora.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- Internal API implementation (to be used in DEVICE_API() *only*) ------------------------------------------------------------
 */
int ral_driver_init(const struct device *dev);

int ral_driver_config(const struct device *dev, struct lora_modem_config *config);
int ral_driver_send(const struct device *dev, uint8_t *data, uint32_t data_len);
int ral_driver_send_async(const struct device *dev, uint8_t *data, uint32_t data_len, struct k_poll_signal *async);
int ral_driver_recv(const struct device *dev, uint8_t *data, uint8_t size, k_timeout_t timeout, int16_t *rssi, int8_t *snr);
int ral_driver_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data);

#ifdef __cplusplus
}
#endif

#endif  // RAL_DRIVER_LEGACY_H
