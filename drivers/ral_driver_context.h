#ifndef RAL_DRIVER_CONTEXT_H
#define RAL_DRIVER_CONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include "ralf.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
struct ralf_hal_config_t {
    ralf_t modem_radio;

	// work to be done after IRQ was raised
	struct k_work work;
};

#ifdef __cplusplus
}
#endif

#endif  // RAL_DRIVER_CONTEXT_H
