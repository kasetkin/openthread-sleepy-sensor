set(srcs
  main.cpp
  common_utils.cpp
  errortask.cpp
  sensorstask.cpp
  matter_sensor.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES 
                            sht3x 
                            nvs_flash 
                            esp_timer esp_driver_gpio esp_driver_usb_serial_jtag esp_event esp_http_client 
                            esp_matter 
                            openthread
                       EMBED_TXTFILES ../secrets.yaml ../calibration.txt)

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 23)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)
