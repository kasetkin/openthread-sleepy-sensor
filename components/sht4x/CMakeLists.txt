if(${IDF_VERSION_MAJOR} STREQUAL 4 AND ${IDF_VERSION_MINOR} STREQUAL 1 AND ${IDF_VERSION_PATCH} STREQUAL 3)
	set(req i2cdev log esp_idf_lib_helpers esp_pm)
else()
	set(req i2cdev log esp_idf_lib_helpers esp_timer esp_pm)
endif()

idf_component_register(
    SRCS sht4x.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)
