// Copyright (c) 2015-2019 The HomeKit ADK Contributors
//
// Licensed under the Apache License, Version 2.0 (the “License”);
// you may not use this file except in compliance with the License.
// See [CONTRIBUTORS.md] for the list of HomeKit ADK project authors.

// An example that implements the light bulb HomeKit profile. It can serve as a basic implementation for
// any platform. The accessory logic implementation is reduced to internal state updates and log output.
//
// This implementation is platform-independent.
//
// The code consists of multiple parts:
//
//   1. The definition of the accessory configuration and its internal state.
//
//   2. Helper functions to load and save the state of the accessory.
//
//   3. The definitions for the HomeKit attribute database.
//
//   4. The callbacks that implement the actual behavior of the accessory, in this
//      case here they merely access the global accessory state variable and write
//      to the log to make the behavior easily observable.
//
//   5. The initialization of the accessory state.
//
//   6. Callbacks that notify the server in case their associated value has changed.

#include "HAP.h"

#include "App.h"
#include "DB.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Domain used in the key value store for application data.
 *
 * Purged: On factory reset.
 */
#define kAppKeyValueStoreDomain_Configuration ((HAPPlatformKeyValueStoreDomain) 0x00)

/**
 * Key used in the key value store to store the configuration state.
 *
 * Purged: On factory reset.
 */
#define kAppKeyValueStoreKey_Configuration_State ((HAPPlatformKeyValueStoreDomain) 0x00)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Global accessory configuration.
 */
typedef struct {
    struct {
        bool lightBulbOn;
    } state;
    HAPAccessoryServerRef* server;
    HAPPlatformKeyValueStoreRef keyValueStore;
} AccessoryConfiguration;

static AccessoryConfiguration accessoryConfiguration;

static pms5003t_data pms5003t_data_info = {
    .PM1_0 = 0,
    .PM2_5 = 0,
    .PM10 = 0,
    .temperature = 0,
    .humidity = 0,
};

static temphum temperature_humidity_value = {
    .temp = 0,
    .hum = 0,
};
//----------------------------------------------------------------------------------------------------------------------

/**
 * Load the accessory state from persistent memory.
 */
static void LoadAccessoryState(void) {
    HAPPrecondition(accessoryConfiguration.keyValueStore);

    HAPError err;

    // Load persistent state if available
    bool found;
    size_t numBytes;

    err = HAPPlatformKeyValueStoreGet(
            accessoryConfiguration.keyValueStore,
            kAppKeyValueStoreDomain_Configuration,
            kAppKeyValueStoreKey_Configuration_State,
            &accessoryConfiguration.state,
            sizeof accessoryConfiguration.state,
            &numBytes,
            &found);

    if (err) {
        HAPAssert(err == kHAPError_Unknown);
        HAPFatalError();
    }
    if (!found || numBytes != sizeof accessoryConfiguration.state) {
        if (found) {
            HAPLogError(&kHAPLog_Default, "Unexpected app state found in key-value store. Resetting to default.");
        }
        HAPRawBufferZero(&accessoryConfiguration.state, sizeof accessoryConfiguration.state);
    }
}

/**
 * Save the accessory state to persistent memory.
 */
static void SaveAccessoryState(void) {
    HAPPrecondition(accessoryConfiguration.keyValueStore);

    HAPError err;
    err = HAPPlatformKeyValueStoreSet(
            accessoryConfiguration.keyValueStore,
            kAppKeyValueStoreDomain_Configuration,
            kAppKeyValueStoreKey_Configuration_State,
            &accessoryConfiguration.state,
            sizeof accessoryConfiguration.state);
    if (err) {
        HAPAssert(err == kHAPError_Unknown);
        HAPFatalError();
    }
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * HomeKit accessory that provides the Light Bulb service.
 *
 * Note: Not constant to enable BCT Manual Name Change.
 */
static HAPAccessory accessory = { .aid = 1,
                                  .category = kHAPAccessoryCategory_Sensors,
                                  .name = "Temperature_Humidity",
                                  .manufacturer = "Bohung",
                                  .model = "Temperature_Humidityt1,1",
                                  .serialNumber = "0FFFFFFF8D2E",
                                  .firmwareVersion = "1",
                                  .hardwareVersion = "1",
                                  .services = (const HAPService* const[]) { &accessoryInformationService,
                                                                            &hapProtocolInformationService,
                                                                            &pairingService,
                                                                            &lightBulbService,
                                                                            &TEMPService,
                                                                            &HumidityService,
                                                                            NULL },
                                  .callbacks = { .identify = IdentifyAccessory } };

//----------------------------------------------------------------------------------------------------------------------

HAP_RESULT_USE_CHECK
HAPError IdentifyAccessory(
        HAPAccessoryServerRef* server HAP_UNUSED,
        const HAPAccessoryIdentifyRequest* request HAP_UNUSED,
        void* _Nullable context HAP_UNUSED) {
    HAPLogInfo(&kHAPLog_Default, "%s", __func__);
    return kHAPError_None;
}

HAP_RESULT_USE_CHECK
HAPError HandleLightBulbOnRead(
        HAPAccessoryServerRef* server HAP_UNUSED,
        const HAPBoolCharacteristicReadRequest* request HAP_UNUSED,
        bool* value,
        void* _Nullable context HAP_UNUSED) {
    *value = accessoryConfiguration.state.lightBulbOn;
    HAPLogInfo(&kHAPLog_Default, "%s: %s", __func__, *value ? "true" : "false");

    return kHAPError_None;
}

HAP_RESULT_USE_CHECK
HAPError HandleLightBulbOnWrite(
        HAPAccessoryServerRef* server,
        const HAPBoolCharacteristicWriteRequest* request,
        bool value,
        void* _Nullable context HAP_UNUSED) {
    HAPLogInfo(&kHAPLog_Default, "%s: %s", __func__, value ? "true" : "false");
    if (accessoryConfiguration.state.lightBulbOn != value) {
        accessoryConfiguration.state.lightBulbOn = value;
        unsigned long sig = 0;
        if(value)
            sig = 1;
        xQueueSend(MsgQueue, &sig, 0);
        SaveAccessoryState();
        HAPAccessoryServerRaiseEvent(server, request->characteristic, request->service, request->accessory);
    }

    return kHAPError_None;
}

HAP_RESULT_USE_CHECK
HAPError HandleCORead(
        HAPAccessoryServerRef* server HAP_UNUSED,
        const HAPFloatCharacteristicReadRequest* request HAP_UNUSED,
        float* value,
        void* _Nullable context){
        *value = (int)(990.0 / adc1_get_raw(ADC1_CHANNEL_6)) + 10;
        // *value = 200.0;
        // printf("%d\n", adc1_get_raw(ADC1_CHANNEL_6));
        return kHAPError_None;
}

HAP_RESULT_USE_CHECK
HAPError HandleCODetectRead(
        HAPAccessoryServerRef* server HAP_UNUSED,
        const HAPUInt8CharacteristicReadRequest* request HAP_UNUSED,
        uint8_t* value,
        void* _Nullable context HAP_UNUSED){
        float density = *value = (int)(990.0 / adc1_get_raw(ADC1_CHANNEL_6)) + 10;
        if(density > 80)
            *value = 1;
        else
            *value = 0;
        return kHAPError_None;
}   

HAP_RESULT_USE_CHECK
HAPError HandleTEMPRead(
        HAPAccessoryServerRef* server HAP_UNUSED,
        const HAPFloatCharacteristicReadRequest* request HAP_UNUSED,
        float* value,
        void* _Nullable context HAP_UNUSED){
        temphum temphumtmp;
        if(xQueueReceive(SHT20_queue, &temphumtmp, 100 / portTICK_RATE_MS) == pdPASS){
            if(temphumtmp.temp < 30 && temphumtmp.temp > 0){
                temperature_humidity_value.temp = temphumtmp.temp;
            }
        }
        *value = temperature_humidity_value.temp;
        return kHAPError_None;
}  

HAP_RESULT_USE_CHECK
HAPError HandleAirQualityRead(
        HAPAccessoryServerRef* server,
        const HAPUInt8CharacteristicReadRequest* request,
        uint8_t* value,
        void* _Nullable context){
        *value = 1;
        return kHAPError_None;
}

HAP_RESULT_USE_CHECK
HAPError HandleAirQualityPM25Read(
        HAPAccessoryServerRef* server,
        const HAPFloatCharacteristicReadRequest* request,
        float* value,
        void* _Nullable context){
        pms5003t_data pms5003t_data_tmp;
        if(xQueueReceive(pms5003t_queue, &pms5003t_data_tmp, 1000 / portTICK_RATE_MS) == pdPASS){
            if(pms5003t_data_tmp.PM2_5 <= 1000 && pms5003t_data_tmp.PM2_5 >= 0){
                pms5003t_data_info.PM2_5 = pms5003t_data_tmp.PM2_5;
            }
        }
        *value = (float)pms5003t_data_info.PM2_5;
        /*
        if(pms5003t_data_info.PM2_5 >= 1000 || pms5003t_data_info.PM2_5 <= 0)
            *value = 0;
        else
            *value = (float)pms5003t_data_info.PM2_5;
        */
        return kHAPError_None;  
}

HAP_RESULT_USE_CHECK
HAPError HandleAirQualityPM10Read(
        HAPAccessoryServerRef* server,
        const HAPFloatCharacteristicReadRequest* request,
        float* value,
        void* _Nullable context){
        pms5003t_data pms5003t_data_tmp;
        if(xQueueReceive(pms5003t_queue, &pms5003t_data_tmp, 1000 / portTICK_RATE_MS) == pdPASS){
            if(pms5003t_data_tmp.PM10 < 1000 && pms5003t_data_tmp.PM10 > 0){
                pms5003t_data_info.PM10 = pms5003t_data_tmp.PM10;
            }
        }
        *value = (float)pms5003t_data_info.PM10;
        return kHAPError_None;
}

HAP_RESULT_USE_CHECK
HAPError HandleHumidityRead(
        HAPAccessoryServerRef* server,
        const HAPFloatCharacteristicReadRequest* request,
        float* value,
        void* _Nullable context){
        temphum temphumtmp;
        if(xQueueReceive(SHT20_queue, &temphumtmp, 100 / portTICK_RATE_MS) == pdPASS){
            if(temphumtmp.hum < 100 && temphumtmp.hum > 0){
                temperature_humidity_value.hum = temphumtmp.hum;
            }
        }
        *value = temperature_humidity_value.hum;
        return kHAPError_None;  
}
//----------------------------------------------------------------------------------------------------------------------

void AccessoryNotification(
        const HAPAccessory* accessory,
        const HAPService* service,
        const HAPCharacteristic* characteristic,
        void* ctx) {
    HAPLogInfo(&kHAPLog_Default, "Accessory Notification");

    HAPAccessoryServerRaiseEvent(accessoryConfiguration.server, characteristic, service, accessory);
}

void AppCreate(HAPAccessoryServerRef* server, HAPPlatformKeyValueStoreRef keyValueStore) {
    HAPPrecondition(server);
    HAPPrecondition(keyValueStore);
    HAPLogInfo(&kHAPLog_Default, "%s", __func__);
    HAPRawBufferZero(&accessoryConfiguration, sizeof accessoryConfiguration);
    accessoryConfiguration.server = server;
    accessoryConfiguration.keyValueStore = keyValueStore;
    LoadAccessoryState();
}

void AppRelease(void) {
}

void AppAccessoryServerStart(void) {
    HAPAccessoryServerStart(accessoryConfiguration.server, &accessory);
}

//----------------------------------------------------------------------------------------------------------------------

void AccessoryServerHandleUpdatedState(HAPAccessoryServerRef* server, void* _Nullable context) {
    HAPPrecondition(server);
    HAPPrecondition(!context);

    switch (HAPAccessoryServerGetState(server)) {
        case kHAPAccessoryServerState_Idle: {
            HAPLogInfo(&kHAPLog_Default, "Accessory Server State did update: Idle.");
            return;
        }
        case kHAPAccessoryServerState_Running: {
            HAPLogInfo(&kHAPLog_Default, "Accessory Server State did update: Running.");
            return;
        }
        case kHAPAccessoryServerState_Stopping: {
            HAPLogInfo(&kHAPLog_Default, "Accessory Server State did update: Stopping.");
            return;
        }
    }
    HAPFatalError();
}

const HAPAccessory* AppGetAccessoryInfo() {
    return &accessory;
}

void AppInitialize(
        HAPAccessoryServerOptions* hapAccessoryServerOptions,
        HAPPlatform* hapPlatform,
        HAPAccessoryServerCallbacks* hapAccessoryServerCallbacks) {
    /*no-op*/
}

void AppDeinitialize() {
    /*no-op*/
}
