idf_component_register(SRCS "rtc_slow_cal_repro.c"
                       PRIV_REQUIRES esp_pm esp_timer esp_hw_support esp_driver_tsens)

if(CONFIG_REPRO_EMULATE_CAL_AFTER_WAKEUP)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=rtc_clk_cal")
endif()
