idf_component_register(SRCS ./app_main.c ./DB.c ./App.c ./ws2812b.c ./wificonfig.c ./wificonnect.c ./sht20.c ./PMS5003T.c
                       INCLUDE_DIRS ".")
add_definitions(-DHAP_LOG_LEVEL=${CONFIG_HAP_LOG_LEVEL})
