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
  rtc_clock_fix.cpp
  die_temp.cpp
  net/network_link.cpp
  net/openthread_link.cpp
  net/wifi_link.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." "net"
                       PRIV_REQUIRES nvs_flash esp_timer esp_adc esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_driver_tsens esp_event openthread espressif__mqtt esp_wifi esp_netif esp_coex mbedtls lp_sensor_core app_update esp_app_format
                       EMBED_TXTFILES ../secrets.yaml ../device_config.yaml
                       WHOLE_ARCHIVE)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 26)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)

# Fix for the device clock running fast in light sleep: hands ESP-IDF's light-sleep path an RTC_SLOW
# calibration taken right after the previous wake instead of at sleep entry -- see rtc_clock_fix.h.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=rtc_clk_cal")
