#include "resetful_api.h"


/**
 * get temperature and humidity
 */
static esp_err_t temperature_humidity_api(httpd_req_t *req)
{

    /* Store information from char to json */
    char temperature_humidity_value_string[200];

    /* Store temperature and humidity from senor */
    static temphum temperature_humidity_value = {
        .temp = 0,
        .hum = 0,
    };

    /* Receive information of temperature and humidity from queue */
    if(xQueueReceive(SHT20_queue, &temperature_humidity_value, 100 / portTICK_RATE_MS) == pdPASS) {
        
        if(temperature_humidity_value.hum > 100 && temperature_humidity_value.hum < 0){
            temperature_humidity_value.hum = 0;
        }

        if(temperature_humidity_value.temp > 30 && temperature_humidity_value.temp < 0) {
            temperature_humidity_value.temp = 0;
        }
    }

    sprintf(temperature_humidity_value_string, "{\"Temperature\":%.2e,\"Humidity\":%.2e}", \
            temperature_humidity_value.temp, temperature_humidity_value.hum);

    /* Send*/
    httpd_resp_send(req, temperature_humidity_value_string, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/**
 * get system information
 */
static esp_err_t system_infotmation_api(httpd_req_t *req)
{
    char buffer[200];
    sprintf(buffer, "{\"PRODUCT_NAME\":\"%s\",\"MANUFACTURER\":\"%s\", \
            \"MODEL\":\"%s\",\"SERIALNUMBER:\":\"%s\"}", \
            PRODUCT_NAME, MANUFACTURER, MODEL, SERIALNUMBER);
    httpd_resp_send(req, buffer, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


/**
 * Register uri for temperature and humidity
 */
static httpd_uri_t temperature_humidity_uri = {
    .uri      = "/tempertature_humidity",
    .method   = HTTP_POST,
    .handler  = temperature_humidity_api,
    .user_ctx = NULL
};

/**
 * Register uri for system information
 */
static httpd_uri_t system_infotmation_uri = {
    .uri      = "/system_infotmation",
    .method   = HTTP_POST,
    .handler  = system_infotmation_api,
    .user_ctx = NULL
};

httpd_handle_t start_webserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Invoke API for temperature and humidity*/
        httpd_register_uri_handler(server, &temperature_humidity_uri);

        /* Invoke API for temperature and humidity*/ 
        httpd_register_uri_handler(server, &system_infotmation_uri);
    }
    /* If server failed to start, handle will be NULL */
    return server;
}