// Copyright (c) 2015-2019 The HomeKit ADK Contributors
//
// Licensed under the Apache License, Version 2.0 (the “License”);
// you may not use this file except in compliance with the License.
// See [CONTRIBUTORS.md] for the list of HomeKit ADK project authors.
// This example code is in the Public Domain (or CC0 licensed, at your option.)
//
// Unless required by applicable law or agreed to in writing, this
// software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied.

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "App.h"
#include "DB.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#define IP 1
#include "HAP.h"
#include "HAPPlatform+Init.h"
#include "HAPPlatformAccessorySetup+Init.h"
#include "HAPPlatformBLEPeripheralManager+Init.h"
#include "HAPPlatformKeyValueStore+Init.h"
#include "HAPPlatformMFiHWAuth+Init.h"
#include "HAPPlatformMFiTokenAuth+Init.h"
#include "HAPPlatformRunLoop+Init.h"
#include "wificonfig.h"
#include "wificonnect.h"
#include "sht20.h"
#include "PMS5003T.h"
#if IP
#include "HAPPlatformServiceDiscovery+Init.h"
#include "HAPPlatformTCPStreamManager+Init.h"
#endif

#include <signal.h>
static bool requestedFactoryReset = false;
static bool clearPairings = false;
#define GPIO_INPUT_IO_0    19 // reset button
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT_IO_0)
#define PREFERRED_ADVERTISING_INTERVAL (HAPBLEAdvertisingIntervalCreateFromMilliseconds(417.5f))

SemaphoreHandle_t SHT20_mutex;

extern void smart_wifi(void);
extern void app_wifi_init(void);
extern esp_err_t app_wifi_connect(wifi_config_t);
/**
 * Global platform objects.
 * Only tracks objects that will be released in DeinitializePlatform.
 */
static struct {
    HAPPlatformKeyValueStore keyValueStore;
    HAPPlatformKeyValueStore factoryKeyValueStore;
    HAPAccessoryServerOptions hapAccessoryServerOptions;
    HAPPlatform hapPlatform;
    HAPAccessoryServerCallbacks hapAccessoryServerCallbacks;

#if HAVE_NFC
    HAPPlatformAccessorySetupNFC setupNFC;
#endif

#if IP
    HAPPlatformTCPStreamManager tcpStreamManager;
#endif

    HAPPlatformMFiHWAuth mfiHWAuth;
    HAPPlatformMFiTokenAuth mfiTokenAuth;
} platform;

/**
 * HomeKit accessory server that hosts the accessory.
 */
static HAPAccessoryServerRef accessoryServer;

void HandleUpdatedState(HAPAccessoryServerRef* _Nonnull server, void* _Nullable context);

/**
 * Functions provided by App.c for each accessory application.
 */
extern void AppRelease(void);
extern void AppCreate(HAPAccessoryServerRef* server, HAPPlatformKeyValueStoreRef keyValueStore);
extern void AppInitialize(
        HAPAccessoryServerOptions* hapAccessoryServerOptions,
        HAPPlatform* hapPlatform,
        HAPAccessoryServerCallbacks* hapAccessoryServerCallbacks);
extern void AppDeinitialize();
extern void AppAccessoryServerStart(void);
extern void AccessoryServerHandleUpdatedState(HAPAccessoryServerRef* server, void* _Nullable context);
extern const HAPAccessory* AppGetAccessoryInfo();

/**
 * Initialize global platform objects.
 */
static void InitializePlatform() {
    // Key-value store.
    HAPPlatformKeyValueStoreCreate(&platform.keyValueStore, &(const HAPPlatformKeyValueStoreOptions) {
        .part_name = "nvs",
        .namespace_prefix = "hap",
        .read_only = false
    });
    platform.hapPlatform.keyValueStore = &platform.keyValueStore;

    HAPPlatformKeyValueStoreCreate(&platform.factoryKeyValueStore, &(const HAPPlatformKeyValueStoreOptions) {
        .part_name = CONFIG_EXAMPLE_FACTORY_PARTITION_NAME,
        .namespace_prefix = "hap",
        .read_only = true
    });

    // Accessory setup manager. Depends on key-value store.
    static HAPPlatformAccessorySetup accessorySetup;
    HAPPlatformAccessorySetupCreate(
            &accessorySetup, &(const HAPPlatformAccessorySetupOptions) { .keyValueStore = &platform.factoryKeyValueStore });
    platform.hapPlatform.accessorySetup = &accessorySetup;
    
    // Initialise Wi-Fi
    unsigned long send_data_temp = 100;
    xQueueSend(MsgQueue, &send_data_temp, 100 / portTICK_RATE_MS);
    wifi_start_connect();
    // wait connect
    while(1){
        EventBits_t uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, true, false, 100 / portTICK_RATE_MS);
        if( (uxBits & CONNECTED_BIT) == CONNECTED_BIT){
            break;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    send_data_temp = 101;
    xQueueSend(MsgQueue, &send_data_temp, 100 / portTICK_RATE_MS);
#if IP
    // TCP stream manager.
    HAPPlatformTCPStreamManagerCreate(&platform.tcpStreamManager, &(const HAPPlatformTCPStreamManagerOptions) {
        /* Listen on all available network interfaces. */
        .port = 0 /* Listen on unused port number from the ephemeral port range. */,
        .maxConcurrentTCPStreams = 9
    });

    // Service discovery.
    static HAPPlatformServiceDiscovery serviceDiscovery;
    HAPPlatformServiceDiscoveryCreate(&serviceDiscovery, &(const HAPPlatformServiceDiscoveryOptions) {
        0, /* Register services on all available network interfaces. */
    });
    platform.hapPlatform.ip.serviceDiscovery = &serviceDiscovery;
#endif

#if (BLE)
    // BLE peripheral manager. Depends on key-value store.
    static HAPPlatformBLEPeripheralManagerOptions blePMOptions = { 0 };
    blePMOptions.keyValueStore = &platform.keyValueStore;

    static HAPPlatformBLEPeripheralManager blePeripheralManager;
    HAPPlatformBLEPeripheralManagerCreate(&blePeripheralManager, &blePMOptions);
    platform.hapPlatform.ble.blePeripheralManager = &blePeripheralManager;
#endif

#if HAVE_MFI_HW_AUTH
    // Apple Authentication Coprocessor provider.
    HAPPlatformMFiHWAuthCreate(&platform.mfiHWAuth);
#endif

#if HAVE_MFI_HW_AUTH
    platform.hapPlatform.authentication.mfiHWAuth = &platform.mfiHWAuth;
#endif

    // Software Token provider. Depends on key-value store.
    HAPPlatformMFiTokenAuthCreate(
            &platform.mfiTokenAuth,
            &(const HAPPlatformMFiTokenAuthOptions) { .keyValueStore = &platform.keyValueStore });

    // Run loop.
    HAPPlatformRunLoopCreate(&(const HAPPlatformRunLoopOptions) { .keyValueStore = &platform.keyValueStore });

    platform.hapAccessoryServerOptions.maxPairings = kHAPPairingStorage_MinElements;

    platform.hapPlatform.authentication.mfiTokenAuth =
            HAPPlatformMFiTokenAuthIsProvisioned(&platform.mfiTokenAuth) ? &platform.mfiTokenAuth : NULL;

   platform.hapAccessoryServerCallbacks.handleUpdatedState = HandleUpdatedState;
}

/**
 * Deinitialize global platform objects.
 */
static void DeinitializePlatform() {
#if HAVE_MFI_HW_AUTH
    // Apple Authentication Coprocessor provider.
    HAPPlatformMFiHWAuthRelease(&platform.mfiHWAuth);
#endif

#if IP
    // TCP stream manager.
    HAPPlatformTCPStreamManagerRelease(&platform.tcpStreamManager);
#endif

    AppDeinitialize();

    // Run loop.
    HAPPlatformRunLoopRelease();
}

/**
 * Restore platform specific factory settings.
 */
void RestorePlatformFactorySettings(void) {
}

/**
 * Either simply passes State handling to app, or processes Factory Reset
 */
void HandleUpdatedState(HAPAccessoryServerRef* _Nonnull server, void* _Nullable context) {
    if (HAPAccessoryServerGetState(server) == kHAPAccessoryServerState_Idle && requestedFactoryReset) {
        HAPPrecondition(server);

        HAPError err;

        HAPLogInfo(&kHAPLog_Default, "A factory reset has been requested.");

        // Purge app state.
        err = HAPPlatformKeyValueStorePurgeDomain(&platform.keyValueStore, ((HAPPlatformKeyValueStoreDomain) 0x00));
        if (err) {
            HAPAssert(err == kHAPError_Unknown);
            HAPFatalError();
        }

        // Reset HomeKit state.
        err = HAPRestoreFactorySettings(&platform.keyValueStore);
        if (err) {
            HAPAssert(err == kHAPError_Unknown);
            HAPFatalError();
        }

        // Restore platform specific factory settings.
        RestorePlatformFactorySettings();

        // De-initialize App.
        AppRelease();

        requestedFactoryReset = false;

        // Re-initialize App.
        AppCreate(server, &platform.keyValueStore);

        // Restart accessory server.
        AppAccessoryServerStart();
        return;
    } else if (HAPAccessoryServerGetState(server) == kHAPAccessoryServerState_Idle && clearPairings) {
        HAPError err;
        err = HAPRemoveAllPairings(&platform.keyValueStore);
        if (err) {
            HAPAssert(err == kHAPError_Unknown);
            HAPFatalError();
        }
        AppAccessoryServerStart();
    } else {
        AccessoryServerHandleUpdatedState(server, context);
    }
}

#if IP
static void InitializeIP() {
    // Prepare accessory server storage.
    static HAPIPSession ipSessions[kHAPIPSessionStorage_MinimumNumElements];
    static uint8_t ipInboundBuffers[HAPArrayCount(ipSessions)][kHAPIPSession_MinimumInboundBufferSize];
    static uint8_t ipOutboundBuffers[HAPArrayCount(ipSessions)][kHAPIPSession_MinimumOutboundBufferSize];
    static HAPIPEventNotificationRef ipEventNotifications[HAPArrayCount(ipSessions)][kAttributeCount];
    for (size_t i = 0; i < HAPArrayCount(ipSessions); i++) {
        ipSessions[i].inboundBuffer.bytes = ipInboundBuffers[i];
        ipSessions[i].inboundBuffer.numBytes = sizeof ipInboundBuffers[i];
        ipSessions[i].outboundBuffer.bytes = ipOutboundBuffers[i];
        ipSessions[i].outboundBuffer.numBytes = sizeof ipOutboundBuffers[i];
        ipSessions[i].eventNotifications = ipEventNotifications[i];
        ipSessions[i].numEventNotifications = HAPArrayCount(ipEventNotifications[i]);
    }
    static HAPIPReadContextRef ipReadContexts[kAttributeCount];
    static HAPIPWriteContextRef ipWriteContexts[kAttributeCount];
    static uint8_t ipScratchBuffer[kHAPIPSession_MinimumScratchBufferSize];
    static HAPIPAccessoryServerStorage ipAccessoryServerStorage = {
        .sessions = ipSessions,
        .numSessions = HAPArrayCount(ipSessions),
        .readContexts = ipReadContexts,
        .numReadContexts = HAPArrayCount(ipReadContexts),
        .writeContexts = ipWriteContexts,
        .numWriteContexts = HAPArrayCount(ipWriteContexts),
        .scratchBuffer = { .bytes = ipScratchBuffer, .numBytes = sizeof ipScratchBuffer }
    };

    platform.hapAccessoryServerOptions.ip.transport = &kHAPAccessoryServerTransport_IP;
    platform.hapAccessoryServerOptions.ip.accessoryServerStorage = &ipAccessoryServerStorage;

    platform.hapPlatform.ip.tcpStreamManager = &platform.tcpStreamManager;
}
#endif

#if BLE
static void InitializeBLE() {
    static HAPBLEGATTTableElementRef gattTableElements[kAttributeCount];
    static HAPBLESessionCacheElementRef sessionCacheElements[kHAPBLESessionCache_MinElements];
    static HAPSessionRef session;
    static uint8_t procedureBytes[2048];
    static HAPBLEProcedureRef procedures[1];

    static HAPBLEAccessoryServerStorage bleAccessoryServerStorage = {
        .gattTableElements = gattTableElements,
        .numGATTTableElements = HAPArrayCount(gattTableElements),
        .sessionCacheElements = sessionCacheElements,
        .numSessionCacheElements = HAPArrayCount(sessionCacheElements),
        .session = &session,
        .procedures = procedures,
        .numProcedures = HAPArrayCount(procedures),
        .procedureBuffer = { .bytes = procedureBytes, .numBytes = sizeof procedureBytes }
    };

    platform.hapAccessoryServerOptions.ble.transport = &kHAPAccessoryServerTransport_BLE;
    platform.hapAccessoryServerOptions.ble.accessoryServerStorage = &bleAccessoryServerStorage;
    platform.hapAccessoryServerOptions.ble.preferredAdvertisingInterval = PREFERRED_ADVERTISING_INTERVAL;
    platform.hapAccessoryServerOptions.ble.preferredNotificationDuration = kHAPBLENotification_MinDuration;
}
#endif

void main_task()
{
    //
     
    HAPAssert(HAPGetCompatibilityVersion() == HAP_COMPATIBILITY_VERSION);

    // Initialize global platform objects.
    InitializePlatform();

#if IP
    InitializeIP();
#endif

#if BLE
    InitializeBLE();
#endif

    // Perform Application-specific initalizations such as setting up callbacks
    // and configure any additional unique platform dependencies
    AppInitialize(&platform.hapAccessoryServerOptions, &platform.hapPlatform, &platform.hapAccessoryServerCallbacks);

    // Initialize accessory server.
    HAPAccessoryServerCreate(
            &accessoryServer,
            &platform.hapAccessoryServerOptions,
            &platform.hapPlatform,
            &platform.hapAccessoryServerCallbacks,
            /* context: */ NULL);

    // Create app object.
    AppCreate(&accessoryServer, &platform.keyValueStore);

    // Start accessory server for App.
    AppAccessoryServerStart();

    // Run main loop until explicitly stopped.
    HAPPlatformRunLoopRun();
    // Run loop stopped explicitly by calling function HAPPlatformRunLoopStop.

    // Cleanup.
    AppRelease();

    HAPAccessoryServerRelease(&accessoryServer);

    DeinitializePlatform();
}

void task_led(void *argument){
    ws2812b_t *led_strip = new_ws2812b(1, GPIO_NUM_18, RMT_CHANNEL_0);
    uint8_t led_color_table[] = {0, 0, 0, 255, 128, 0, 0, 255, 128, 0, 0 ,255, 128, 128 ,0, 0, 128, 128};
    uint8_t alarm_led[] = {0, 255, 0, 255 ,0 ,0};
    uint8_t *ptr_led_color  = led_color_table;
    uint8_t *ptr_alarm_led = alarm_led;
    uint8_t *ptr_led = NULL;
    uint32_t rcv = 0;
    uint8_t light_idx = 0;
    while(1){
        rcv = 0;
        if(xQueueReceive(MsgQueue, &rcv, 1000 / portTICK_RATE_MS) == pdPASS){
            if(rcv == 100 || rcv == 101){
                ptr_led = ptr_alarm_led + (rcv - 100) * 3;
            }
            else { 
                if(rcv){
                    uint8_t led_offset;
                    if(light_idx > (sizeof(led_color_table) - sizeof(uint8_t)) / 3)
                        light_idx = 1;
                    light_idx++;
                    led_offset = (3 * light_idx);
                    ptr_led_color = led_color_table + led_offset;
                }
                else
                {
                    ptr_led_color = led_color_table;
                }
                ptr_led = ptr_led_color;
            }
        }
        set_pixel(led_strip, ptr_led);
        vTaskDelay(pdMS_TO_TICKS(5));
        led_flush(led_strip);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void reset_func(void *argument){
    while(1){
        if(!gpio_get_level(GPIO_INPUT_IO_0)){
            ESP_LOGI("Reset", "reset apple homekit paired");
            spi_flash_erase_range(0x10000, 0x1000);
            vTaskDelay(20 / portTICK_RATE_MS);
            esp_restart();
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void temphum_task(void* argument){
    while(1){
        if(xSemaphoreTake(SHT20_mutex, (TickType_t)10) == pdTRUE){
            temphum temphumobj = readtemphum();
            // printf("%e %e\n", temphumobj.hum, temphumobj.temp);
            xQueueSend(SHT20_queue, &temphumobj, 100 / portTICK_RATE_MS);
            xSemaphoreGive(SHT20_mutex);
        }    
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void pms5003t(void* argument){
    pms5003t_data *new_pms5003t_data = pms5003t_initial(2);
    pms5003t_data pms5003t_data_ll;
    while(1){
        pms5003t_read(new_pms5003t_data);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    // spi_flash_init();
    wificonfig_initial();
    MsgQueue = xQueueCreate(10, sizeof(unsigned long));
    // reset
    gpio_config_t io_conf_reset = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .pull_down_en = 0,
    };
    gpio_config(&io_conf_reset);
    // ADC initial
    //adc1_config_width(ADC_WIDTH_BIT_12);
    //adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    sht20_initial(22, 21);
    SHT20_mutex = xSemaphoreCreateMutex();
    SHT20_queue = xQueueCreate(10, sizeof(temphum));
    // xTaskCreate(reset_func, "reset_func", 6 * 1024, NULL, 6, NULL);
    xTaskCreate(main_task, "main_task", 6 * 1024, NULL, 5, NULL);
    xTaskCreate(task_led, "task_led", 6 * 1024, NULL, 7, NULL);
    xTaskCreate(temphum_task, "temphum_task", 6 * 1024, NULL, 8, NULL);
}