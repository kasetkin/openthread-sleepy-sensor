idf_component_register(
    SRCS "lp_sensor_core.c"
    INCLUDE_DIRS "include"
    REQUIRES ulp esp_hw_support esp_pm freertos
)

# --- ULP LP-core sub-build (see ESP-IDF's examples/system/ulp/lp_core/lp_i2c for the
#     reference pattern this mirrors) ---

# Must be unique if multiple components ever embed an LP core program.
set(ulp_app_name lp_core_${COMPONENT_NAME})

# LP-only sources, kept out of the component's own SRCS above.
set(ulp_lp_core_sources "lp_core/main.cpp")

# HP-side source(s) that #include the auto-generated ${ulp_app_name}.h export header.
set(ulp_exp_dep_srcs "lp_sensor_core.c")

ulp_embed_binary(${ulp_app_name} "${ulp_lp_core_sources}" "${ulp_exp_dep_srcs}")
