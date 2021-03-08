idf_component_register(SRCS ./app_main.c ./DB.c ./App.c ./wificonfig.c ./wificonnect.c ./sht20.c
                       INCLUDE_DIRS ".")
add_definitions(-DHAP_LOG_LEVEL=${CONFIG_HAP_LOG_LEVEL})
