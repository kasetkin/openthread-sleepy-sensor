set(srcs
  main.cpp
  common_utils.cpp
  errortask.cpp
  sensorstask.cpp
  mqtt_sender.cpp
  net/network_link.cpp
  net/openthread_link.cpp
  net/wifi_link.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." "net"
                       PRIV_REQUIRES sht3x sht4x esp_driver_tsens nvs_flash esp_timer esp_driver_gpio esp_driver_usb_serial_jtag esp_event openthread espressif__mqtt esp_wifi esp_netif esp_coex mbedtls
                       EMBED_TXTFILES ../secrets.yaml ../calibration.txt
                       WHOLE_ARCHIVE)
#  esp_adc -- maybe for battery reading
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 26)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)
