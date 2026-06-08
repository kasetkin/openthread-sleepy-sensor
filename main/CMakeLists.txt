set(srcs 
  main.cpp 
  common_utils.cpp 
  errortask.cpp
  sensorstask.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES sht3x nvs_flash esp_timer esp_driver_gpio esp_event openthread 
                       WHOLE_ARCHIVE)
#  esp_adc -- maybe for battery reading
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 26)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)
