#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>

#include "rs485.h"

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define MAX_DATA_LEN 10
char data[MAX_DATA_LEN] = {'h', 'h', 'i', 'O', 'r', '1', 'o', 'r', 'l', 'd'};

int main(void) {
  LOG_INIT();
  
  const struct device *const lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
  struct lora_modem_config config;
  int ret;

  if (!device_is_ready(lora_dev)) {
    LOG_ERR("%s Device not ready", lora_dev->name);
    return 0;
  }

  config.frequency = 865000000;
  config.bandwidth = BW_125_KHZ;
  config.datarate = SF_8;
  config.preamble_len = 12;
  config.coding_rate = CR_4_5;
  config.iq_inverted = false;
  config.public_network = false;
  config.tx_power = 16;
  config.tx = true;

  ret = lora_config(lora_dev, &config);
  if (ret < 0) {
    LOG_ERR("LoRa config failed");
    return 0;
  }

  data[0] = '0';
  data[5] = '0';
  LOG_INF("Ready to start sending");

  while (1) {
    ret = lora_send(lora_dev, data, MAX_DATA_LEN);
    if (ret < 0) {
            LOG_ERR("LoRa send failed");
            return 0;
    }

    LOG_INF("Data sent!");

    /* Send data at 3s interval */
    k_sleep(K_MSEC(3000));

    data[0]++; data[5]++;
    if (data[5] > '9') { data[0] = '0'; data[5] = '0'; }
  }

	return 0;
}
