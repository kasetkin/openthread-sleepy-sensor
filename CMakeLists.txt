# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(openthread-sleepy-sensor)

# Enable OpenThread's Link Metrics INITIATOR role, which ESP-IDF's Kconfig cannot reach on an
# MTD build. CONFIG_OPENTHREAD_LINK_METRICS=y (sdkconfig.defaults) looks like it covers this,
# but the option is only consumed by openthread-core-esp32x-ftd-config.h and
# ...-radio-config.h; the MTD config header this build actually uses never mentions link
# metrics at all, so OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE keeps OpenThread's
# own default of 0 and every link-metrics symbol compiles out. Verified in IDF by the linker:
# without this, otLinkMetricsConfigEnhAckProbing is an undefined reference even with the
# Kconfig option set. (The Kconfig option is still worth setting -- it also pulls in the
# IEEE802154/OpenThread timing optimizations enhanced ACKs need.)
#
# We need only the INITIATOR half: we ask the parent to measure US and report back in its
# enhanced ACKs, which is the sole way this device can observe its own uplink margin (see
# LinkStats::uplinkRssiDbm). The SUBJECT half is what the parent implements, not us.
#
# Scoped to the openthread component rather than set globally via COMPILE_DEFINITIONS: only
# libopenthread's own translation units test this macro. openthread/link_metrics.h declares
# otLinkMetricsConfigEnhAckProbing unconditionally, so main/ compiles fine without it.
idf_component_get_property(openthread_lib openthread COMPONENT_LIB)
target_compile_definitions(${openthread_lib} PRIVATE OPENTHREAD_CONFIG_MLE_LINK_METRICS_INITIATOR_ENABLE=1)
