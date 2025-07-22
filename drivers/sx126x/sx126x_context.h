#ifndef SX126X_CONTEXT_H
#define SX126X_CONTEXT_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <ral_sx126x_bsp.h>

#include <sx126x.h>

#include "ral_driver_context.h"

#ifdef __cplusplus
extern "C" {
#endif

struct sx126x_hal_config_t {
	struct spi_dt_spec spi;        /* spi peripheral */

	struct gpio_dt_spec reset;     /* reset pin */
	struct gpio_dt_spec busy;      /* busy pin */

	struct gpio_dt_spec dio1;      /* DIO1 pin */
	struct gpio_dt_spec rx_enable; /* Rx-enable pin */

	bool dio2_as_rf_switch;

	// Oscilator type
	ral_xosc_cfg_t xosc_cfg;

	// TXCO configuration (if exists)
	sx126x_tcxo_ctrl_voltages_t tcxo_voltage;
	uint32_t tcxo_wakeup_time_ms;

	uint8_t capa_xta; /* set to 0xFF if not configured*/
	uint8_t capa_xtb; /* set to 0xFF if not configured*/

	// LDO or DCDC ?
	sx126x_reg_mod_t reg_mode;

	bool rx_boosted; /* RXBoosted option */

	uint8_t tx_offset; /* Board TX power offset */
};

typedef enum {
	RADIO_SLEEP,
	RADIO_AWAKE
} radio_sleep_status_t;

struct sx126x_hal_data_t {
	// ral binding (must be first field for casting)
	struct ralf_hal_config_t ralf;

	// ** BUSY handling **
	// semaphore for signaling state change
	struct k_sem busy_sem;

	// callback for interrupt
	struct gpio_callback busy_cb;

	// ** DIO1 handling **
	// callback for interrupt
	struct gpio_callback dio1_cb;

	// current radio sleep status
	radio_sleep_status_t radio_status;
};

#ifdef __cplusplus
}
#endif

#endif /* SX126X_CONTEXT_H */
