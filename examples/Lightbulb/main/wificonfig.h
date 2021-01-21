#ifndef _WIFICONFIG_H_
#define _WIFICONFIG_H_
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <freertos/queue.h>
#include <string.h>
#define NVS_CUSTOMER "wifiregion"
#define NVS_REGION "wifidata"

QueueHandle_t netwrok_connect_signal;
wifi_config_t wificonfig_read(void);
void wificonfig_write(wifi_config_t);
void wificonfig_initial(void);
void wifireset(void);

#endif