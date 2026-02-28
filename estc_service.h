#ifndef ESTC_SERVICE_H__
#define ESTC_SERVICE_H__

#include <stdint.h>

#include "ble.h"
#include "sdk_errors.h"

// TODO: 1. Generate random BLE UUID (Version 4 UUID) and define it in the following format:
#define ESTC_BASE_UUID { 0x89, 0xED, 0x3C, 0x19, 0x84, 0x1A, /* - */ 0x31, 0x97, /* - */ 0x09, 0x42, /* - */ 0x2E, 0x32, /* - */ 0x00, 0x00, 0xA3, 0x3D } // UUID: 3da39048-322e-4209-9731-1a84193ced89

// TODO: 2. Pick a random service 16-bit UUID and define it:
#define ESTC_SERVICE_UUID 0x6577

typedef struct
{
    uint16_t service_handle;
} ble_estc_service_t;

ret_code_t estc_ble_service_init(ble_estc_service_t *service);

#endif /* ESTC_SERVICE_H__ */