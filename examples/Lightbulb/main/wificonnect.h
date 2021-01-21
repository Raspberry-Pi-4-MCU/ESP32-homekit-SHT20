#ifndef _WIFICONNECT_H_
#define _WIFICONNECT_H_
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "wificonfig.h"
#include "esp_smartconfig.h"

static const int CONNECTED_BIT = BIT0;
EventGroupHandle_t s_wifi_event_group;
void wifi_start_connect();
#endif