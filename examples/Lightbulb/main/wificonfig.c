#include "wificonfig.h"

wifi_config_t wificonfig_read(void)
{
    // write
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(NVS_CUSTOMER, NVS_READWRITE, &handle));
    // key is exist or not
    size_t size_name = 0;
    nvs_get_used_entry_count(handle, &size_name);
    if(size_name < 5){
        // send fake wifi info
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = "87654321",
                .password = "12345678",
            },
        };
        return wifi_config;
    }
    // read
    wifi_config_t wifi_config_stored;
    memset(&wifi_config_stored, 0x0, sizeof(wifi_config_stored));
    uint32_t len = sizeof(wifi_config_stored);
    ESP_ERROR_CHECK(nvs_get_blob(handle, NVS_REGION, &wifi_config_stored, &len));
    nvs_close(handle);
    return wifi_config_stored;
}

void wificonfig_write(wifi_config_t wifi_config)
{
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(NVS_CUSTOMER, NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_blob(handle, NVS_REGION, &wifi_config, sizeof(wifi_config)));
    nvs_close(handle);
}

void wificonfig_initial(void)
{
    netwrok_connect_signal = xQueueCreate(10, sizeof(int));
}

void wifireset(void){
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "87654321",
            .password = "12345678",
        },
    };
    wificonfig_write(wifi_config);
}