set(srcs
  main.cpp
  common_utils.cpp
  errortask.cpp
  sensorstask.cpp
  mqtt_sender.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES sht3x nvs_flash esp_timer esp_driver_gpio esp_driver_usb_serial_jtag esp_event openthread espressif__mqtt
                       EMBED_TXTFILES ../secrets.yaml
                       WHOLE_ARCHIVE)
#  esp_adc -- maybe for battery reading
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 26)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)
