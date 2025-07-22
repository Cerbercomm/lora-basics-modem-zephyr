cmake_minimum_required(VERSION 3.20.0)

# Nothing to do if we dont have a designated radio driver
if(${CONFIG_HAS_LORA_BASICS_MODEM_RADIO_DRIVER})

  # Add library named after containing folder
  zephyr_library()

  # Module include
  zephyr_include_directories(include)

  # Code for PHY LoRa layer
  add_subdirectory(drivers)

endif()