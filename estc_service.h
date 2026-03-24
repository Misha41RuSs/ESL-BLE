#ifndef ESTC_SERVICE_H__
#define ESTC_SERVICE_H__

#include <stdint.h>

#include "ble.h"
#include "sdk_errors.h"

// TODO: 1. Generate random BLE UUID (Version 4 UUID) and define it in the following format:
#define ESTC_BASE_UUID { 0x89, 0xED, 0x3C, 0x19, 0x84, 0x1A, /* - */ 0x31, 0x97, /* - */ 0x09, 0x42, /* - */ 0x2E, 0x32, /* - */ 0x00, 0x00, 0xA3, 0x3D } // UUID: 3da39048-322e-4209-9731-1a84193ced89

// TODO: 2. Pick a random service 16-bit UUID and define it:
#define ESTC_SERVICE_UUID 0x6577

#define ESTC_GATT_CHAR_LED_STATE_UUID 0x6578   // Read/Write/Notify 
#define ESTC_GATT_CHAR_LED_COLOR_UUID 0x6579   // Read/Write/Notify 

typedef enum {
    BLE_ESTC_EVT_LED_STATE_WRITE,
    BLE_ESTC_EVT_LED_COLOR_WRITE
} ble_estc_evt_type_t;

typedef struct {
    ble_estc_evt_type_t evt_type;
    union {
        uint8_t  led_state; 
        uint32_t led_color; 
    } params;
} ble_estc_evt_t;

struct ble_estc_service_s;
typedef void (*ble_estc_evt_handler_t)(struct ble_estc_service_s *service, ble_estc_evt_t *evt);

typedef struct ble_estc_service_s
{
    uint16_t service_handle;
    uint16_t connection_handle;
    uint8_t  uuid_type;

    ble_gatts_char_handles_t led_state_handles;
    ble_gatts_char_handles_t led_color_handles;

    ble_estc_evt_handler_t evt_handler;
} ble_estc_service_t;

ret_code_t estc_ble_service_init(ble_estc_service_t *service);

void estc_ble_service_on_ble_event(const ble_evt_t *ble_evt, void *ctx);

void estc_update_led_state(ble_estc_service_t *service, uint8_t state);
void estc_update_led_color(ble_estc_service_t *service, uint32_t color);

#endif /* ESTC_SERVICE_H__ */