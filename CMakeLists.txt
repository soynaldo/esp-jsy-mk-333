idf_component_register(SRCS "jsymk333.c" "jsy_crc.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "priv_include"
                       REQUIRES esp_driver_uart esp_timer)