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
#include "driver/periph_ctrl.h"
#include "freertos/queue.h"
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
#include "resetful_api.h"
#include "sht20.h"

#if IP
#include "HAPPlatformServiceDiscovery+Init.h"
#include "HAPPlatformTCPStreamManager+Init.h"
#endif

#include <signal.h>
static bool requestedFactoryReset = false;
static bool clearPairings = false;

/*  reset pin */
#define GPIO_INPUT_IO_0    19
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT_IO_0)
#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_IO_0)

/* Mutex for SHT20 */
SemaphoreHandle_t SHT20_mutex;

/* Queue for system led */
QueueHandle_t sys_led_queue;

#define PREFERRED_ADVERTISING_INTERVAL (HAPBLEAdvertisingIntervalCreateFromMilliseconds(417.5f))

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
    wifi_start_connect();

    // Send signal for waiting
    int32_t sys_led_status = 1;
    xQueueSend(sys_led_queue, &sys_led_status, (TickType_t)10);

    // Waiting for connecting
    while(1) {
        EventBits_t uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, true, false, 100 / portTICK_RATE_MS);
        if( (uxBits & CONNECTED_BIT) == CONNECTED_BIT){
            break;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    // Send signal completed
    sys_led_status = 2;
    xQueueSend(sys_led_queue, &sys_led_status, (TickType_t)10);

    start_webserver();

    // httpd_config_t config = HTTPD_DEFAULT_CONFIG();

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

/**
 * System led
 * Blink: Waiting for connecting
 * Bright: Power on
 * Dark: Power off
 */
void sys_led(void *argument) 
{
    int32_t bright_delay_time = 0;
    int32_t dark_delay_time = 0;
    int32_t sys_led_status_tmp = 0;
    
    while(1) {
        /* Get delay time */
        if(xQueueReceive(sys_led_queue, &(sys_led_status_tmp), (TickType_t)10) == pdPASS ) {
            switch(sys_led_status_tmp) {
            /* Blink */
            case 1:
                bright_delay_time = 200;
                dark_delay_time = 200;
                break;
            /* Bright */
            case 2:
                bright_delay_time = 1000;
                dark_delay_time = 0;
                break;
            /* Bright */
            default:
                bright_delay_time = 1000;
                dark_delay_time = 0;
                break;
            }
        }   

        /* Excute */
        gpio_set_level(GPIO_NUM_18, 1);
        vTaskDelay(bright_delay_time / portTICK_RATE_MS);
        gpio_set_level(GPIO_NUM_18, 0);
        vTaskDelay(dark_delay_time / portTICK_RATE_MS);
    }
}

void reset_func(void *argument)
{
    while(1) {
        /**
         * GPIO 19 as input
         */ 
        if(!gpio_get_level(GPIO_INPUT_IO_0))
        {
            /* Erase wifi information and homekit key from nvs flash */
            spi_flash_erase_range(0x10000, 0x1000);

            /* Waiting for processing */
            vTaskDelay(20 / portTICK_RATE_MS);

            /* reboot after erasing */
            esp_restart();
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

/**
 * Read temperature and humidity from SHT20, and sending.
 */ 
void temphum_task(void* argument)
{
    temphum temphumobj;

    while(1) {
        /* Avoid entering repeatedly */
        if(xSemaphoreTake(SHT20_mutex, (TickType_t)10) == pdTRUE) {
            /* Read temperature and humidity from sensor */
            temphumobj = readtemphum();
            
            /* Send temperature and humidity to homekit */
            xQueueSend(SHT20_queue, &temphumobj, 100 / portTICK_RATE_MS);

            /*  Unlock mutex */
            xSemaphoreGive(SHT20_mutex);
        } 

        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void app_main()
{
    
    /* Initial NVS flash */
    ESP_ERROR_CHECK(nvs_flash_init());
    
    /* Initial WIFI config*/
    wificonfig_initial();

    /**
     * Initial queue for system led
     * Length 5
     */ 
    sys_led_queue = xQueueCreate(5, sizeof(int32_t));

    /**
     * Configure reset pin 
     * GPIO 19 as input pin
     * not use interrupt
     */
    gpio_config_t io_conf_reset = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .pull_down_en = 0,
    };
    gpio_config(&io_conf_reset);
    
    /**
     * Configure led
     * GPIO 18 as input pin
     * not use interrupt
     */
    gpio_config_t io_conf_led = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = 0,
    };
    gpio_config(&io_conf_led);

    /**
     *  Configure SHT20 device
     *  GPIO 22 as SCL
     *  GPIO 21 as SDA 
     */
    sht20_initial(22, 21);
    
    /* Mutex SHT20 */
    SHT20_mutex = xSemaphoreCreateMutex();

    /** 
     * Create SHT20 queue
     * Length 10
     */
    SHT20_queue = xQueueCreate(10, sizeof(temphum));

    /* Provide press event as reset function */
    // xTaskCreate(reset_func, "reset_func", 6 * 1024, NULL, 6, NULL);
    
    xTaskCreate(main_task, "main_task", 6 * 1024, NULL, 5, NULL);

    xTaskCreate(sys_led, "sys_led", 6 * 1024, NULL, 9, NULL);

    /* Read temperature and humidity from SHT20 sensor */
    xTaskCreate(temphum_task, "temphum_task", 6 * 1024, NULL, 8, NULL);
}