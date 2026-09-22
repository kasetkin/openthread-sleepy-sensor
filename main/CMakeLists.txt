set(srcs
  main.cpp
  common_utils.cpp
  errortask.cpp
  sensorstask.cpp
  history_log.cpp
  mqtt_sender.cpp
  ota_updater.cpp
  runtime_config.cpp
  hp_awake_stats.cpp
  rtc_cal_diag.cpp
  net/network_link.cpp
  net/openthread_link.cpp
  net/wifi_link.cpp
  net/ieee802154_rx_at_fix.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." "net"
                       PRIV_REQUIRES nvs_flash esp_timer esp_adc esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_event openthread ieee802154 espressif__mqtt esp_wifi esp_netif esp_coex mbedtls lp_sensor_core app_update esp_app_format
                       EMBED_TXTFILES ../secrets.yaml ../device_config.yaml
                       WHOLE_ARCHIVE)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 26)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)

# Workarounds for ESP-IDF 802.15.4 driver bugs that make a CSL receiver deaf after its own data
# polls, plus a backport of OpenThread #13472 through ot::Mac::SubMac::Sleep() and
# ot::Radio::Statistics::RecordStateChange() (mangled below) that stops the radio listening after
# the MAC goes idle and records that in OpenThread's radio time statistics -- see
# net/ieee802154_rx_at_fix.cpp. Remove together with that file. Once ESP-IDF's OpenThread
# includes #13472, PART C can go on its own -- its code in that file together with both of its
# flags below, or the link breaks.
target_link_libraries(${COMPONENT_LIB} INTERFACE
                      "-Wl,--wrap=esp_ieee802154_receive"
                      "-Wl,--wrap=esp_ieee802154_receive_at"
                      "-Wl,--wrap=_ZN2ot3Mac6SubMac5SleepEv"
                      "-Wl,--wrap=_ZN2ot5Radio10Statistics17RecordStateChangeENS1_6StatusE")

# Diagnostic for the device clock running fast in light sleep: a pass-through wrap that records
# the RTC_SLOW calibrations the light-sleep path makes -- see rtc_cal_diag.h. Remove together with
# that module.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=rtc_clk_cal")
