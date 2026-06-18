cmake_minimum_required(VERSION 3.22)

#if(NOT DEFINED ENV{ESP_MATTER_PATH})
#    message(FATAL_ERROR "Please set ESP_MATTER_PATH (source esp-matter export.sh / use the esp-matter devcontainer)")
#endif()
#
#
#include(${ESP_MATTER_PATH}/examples/common/cmake_common/components_include.cmake)
#
## NOTE: deliberately NOT including esp_matter_device.cmake / the device_hal. It selects the C6
## devkit BSP — ws2812 led_driver (→ espressif/led_strip → led_strip_rmt_ws2812.c) and iot_button
## — which this project doesn't use (we drive our own GPIO + LED in common_utils).
##
## esp-matter + connectedhomeip components. The project's own components/ dir
## (sht3x, i2cdev, matter_glue, …) is auto-discovered by ESP-IDF.

set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
set(MATTER_SDK_PATH ${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)


set(EXTRA_COMPONENT_DIRS
    "${MATTER_SDK_PATH}/config/esp32/components"   # chip + Matter platform components
    "${ESP_MATTER_PATH}/components")               # esp_matter


idf_build_set_property(MINIMAL_BUILD ON)

project(openthread-sleepy-sensor)

# GCC 14.2 (this toolchain) flags connectedhomeip's intentional strncpy truncation in
# Platform::CopyString as an error on the platform-mDNS discovery path (Discovery_ImplPlatform.cpp).
# The CHIP GN build inherits these build-level compile options (see config/esp32/components/chip
# CMakeLists.txt), and IDF's options are applied after CHIP's -Werror, so this suppresses it.

idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++23;-O2;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual;-Wno-error=stringop-truncation;-Wno-error=stringop-overflow;-Wno-format-nonliteral;-Wno-format-security" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-O2" APPEND)
# For RISCV chips, project_include.cmake sets -Wno-format, but does not clear various
# flags that depend on -Wformat
