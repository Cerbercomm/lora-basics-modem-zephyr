# If we're here, we have a designated radio driver

# Some shortcuts to interesting directories
set(LORA_BASICS_MODEM_DIR ${ZEPHYR_CURRENT_MODULE_DIR})
set(LORA_BASICS_MODEM_LIB_DIR ${LORA_BASICS_MODEM_DIR}/lbm_lib)
set(LBM_SMTC_MODEM_CORE_DIR ${LORA_BASICS_MODEM_LIB_DIR}/smtc_modem_core)
set(LBM_RADIO_DRIVERS_DIR ${LORA_BASICS_MODEM_LIB_DIR}/smtc_modem_core/radio_drivers)

# Common includes
zephyr_include_directories(
    ${LBM_SMTC_MODEM_CORE_DIR}/smtc_ral/src
    ${LBM_SMTC_MODEM_CORE_DIR}/smtc_ralf/src

    ${CMAKE_CURRENT_LIST_DIR}
)

# Common source
if (${CONFIG_LORA_BASICS_MODEM_P2P_API_LEGACY})
    zephyr_library_sources(
        ${CMAKE_CURRENT_LIST_DIR}/ral_driver_legacy.c
    )
endif()
if (${CONFIG_LORA_BASICS_MODEM_P2P_API_MODERN})
    zephyr_library_sources(
        ${CMAKE_CURRENT_LIST_DIR}/ral_driver.c
    )
endif()

# Subdirectories specific to each radio driver
add_subdirectory_ifdef(CONFIG_LORA_BASICS_MODEM_SX126X ${CMAKE_CURRENT_LIST_DIR}/sx126x)
