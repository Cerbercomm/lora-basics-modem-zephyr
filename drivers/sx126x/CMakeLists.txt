# SX126x specific cmake stuff

# Used by LBM
zephyr_compile_definitions(SX126X)
zephyr_compile_definitions(SX126X_TRANSCEIVER)
zephyr_compile_definitions(SX126X_DISABLE_WARNINGS)
zephyr_compile_definitions_ifdef(CONFIG_DT_HAS_SEMTECH_SX1261_ENABLED SX1261)
zephyr_compile_definitions_ifdef(CONFIG_DT_HAS_SEMTECH_SX1262_ENABLED SX1262)
zephyr_compile_definitions_ifdef(CONFIG_DT_HAS_SEMTECH_SX1268_ENABLED SX1268)

# Shortcut to our driver directory
set(LBM_SX126X_LIB_DIR ${LBM_RADIO_DRIVERS_DIR}/sx126x_driver/src)

# Includes
zephyr_include_directories(${LBM_SX126X_LIB_DIR})

# Sources from the LBM repository
zephyr_library_sources(
  ${LBM_SX126X_LIB_DIR}/sx126x.c
  ${LBM_SX126X_LIB_DIR}/sx126x_lr_fhss.c
  ${LBM_SX126X_LIB_DIR}/lr_fhss_mac.c
  ${LBM_SX126X_LIB_DIR}/sx126x_driver_version.c
  ${LBM_SMTC_MODEM_CORE_DIR}/smtc_ral/src/ral_sx126x.c
  ${LBM_SMTC_MODEM_CORE_DIR}/smtc_ralf/src/ralf_sx126x.c
)

# HAL & BSP (Hardware Abstraction Layer & Board Support Package) implmenetations for Zephyr
zephyr_library_sources(
  ${CMAKE_CURRENT_LIST_DIR}/sx126x_hal.c
  ${CMAKE_CURRENT_LIST_DIR}/sx126x_ral_bsp.c
)

# Zephyr driver
if (${CONFIG_LORA_BASICS_MODEM_P2P_API_LEGACY})
    zephyr_library_sources(
        ${CMAKE_CURRENT_LIST_DIR}/sx126x_driver_legacy.c
    )
endif()
if (${CONFIG_LORA_BASICS_MODEM_P2P_API_MODERN})
    zephyr_library_sources(
        ${CMAKE_CURRENT_LIST_DIR}/sx126x_driver.c
    )
endif()
