#include "wificonnect.h"

const char *TAG = "WIFI_CONNECT";
static const int ESPTOUCH_DONE_BIT = BIT1;
static void smartconfig_example_task(void * parm);

static void event_handle(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    // wifi event
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
        // read wificonnfig from nvs
        wifi_config_t wificonfig;
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        wificonfig = wificonfig_read();
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wificonfig));
        ESP_ERROR_CHECK(esp_wifi_connect());

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
        ESP_LOGW(TAG, "Connect to the AP failed. switch smart. ");
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
        
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");
        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };
        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }
        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_connect());
        // write NVS
        wificonfig_write(wifi_config);
    }else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

void wifi_start_connect()
{
    // wifi initial
    esp_event_loop_create_default();
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // wifi flag initial
    s_wifi_event_group = xEventGroupCreate();
    // register event
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handle,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handle,
                                                        NULL,
                                                        &instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, 
                                                ESP_EVENT_ANY_ID, 
                                                &event_handle, 
                                                NULL));
    // wifi start
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_smartconfig_stop());
            vTaskDelay(5000 / portTICK_RATE_MS);
            spi_flash_erase_range(0x10000, 0x1000);
            vTaskDelay(200 / portTICK_RATE_MS);
            esp_restart();
            vTaskDelete(NULL);
        }
    }
}