#ifndef ESTC_SERVICE_H__
#define ESTC_SERVICE_H__

#include <stdint.h>

#include "ble.h"
#include "sdk_errors.h"

// TODO: 1. Generate random BLE UUID (Version 4 UUID) and define it in the following format:
#define ESTC_BASE_UUID { 0x89, 0xED, 0x3C, 0x19, 0x84, 0x1A, /* - */ 0x31, 0x97, /* - */ 0x09, 0x42, /* - */ 0x2E, 0x32, /* - */ 0x00, 0x00, 0xA3, 0x3D } // UUID: 3da39048-322e-4209-9731-1a84193ced89

// TODO: 2. Pick a random service 16-bit UUID and define it:
#define ESTC_SERVICE_UUID 0x6577

// TODO: 3. Pick characteristic UUIDs and define them:
#define ESTC_GATT_CHAR_1_UUID 0x6578   // Read/Write 
#define ESTC_GATT_CHAR_2_UUID 0xA7B2   // Notify     
#define ESTC_GATT_CHAR_3_UUID 0xC3F4   // Indicate   

typedef struct
{
    uint16_t service_handle;
    uint16_t connection_handle;
    uint8_t  uuid_type;

    ble_gatts_char_handles_t char1_handles;  // Read/Write characteristic
    ble_gatts_char_handles_t char2_handles;  // Notify characteristic
    ble_gatts_char_handles_t char3_handles;  // Indicate characteristic
} ble_estc_service_t;

ret_code_t estc_ble_service_init(ble_estc_service_t *service);

void estc_ble_service_on_ble_event(const ble_evt_t *ble_evt, void *ctx);

void estc_update_characteristic_1_value(ble_estc_service_t *service, int32_t *value);
void estc_update_characteristic_2_value(ble_estc_service_t *service, int32_t *value);
void estc_update_characteristic_3_value(ble_estc_service_t *service, int32_t *value);

#endif /* ESTC_SERVICE_H__ */