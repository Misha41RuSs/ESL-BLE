#include "estc_service.h"

#include "app_error.h"
#include "nrf_log.h"

#include "ble.h"
#include "ble_gatts.h"
#include "ble_srv_common.h"

static ret_code_t estc_ble_add_characteristics(ble_estc_service_t *service);

ret_code_t estc_ble_service_init(ble_estc_service_t *service)
{
    ret_code_t error_code = NRF_SUCCESS;

    ble_uuid_t    service_uuid;
    ble_uuid128_t base_uuid = {ESTC_BASE_UUID};

    // Add service UUID to the BLE stack vendor-specific UUID table
    error_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(error_code);
    service->uuid_type = service_uuid.type;

    service_uuid.uuid = ESTC_SERVICE_UUID;

    // Register the primary service with the BLE stack
    error_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                          &service_uuid,
                                          &service->service_handle);
    APP_ERROR_CHECK(error_code);

    service->connection_handle = BLE_CONN_HANDLE_INVALID;
    service->is_led_state_notifying = false;
    service->is_led_color_notifying = false;

    return estc_ble_add_characteristics(service);
}


static void cccd_md_init(ble_gatts_attr_md_t *p_cccd_md)
{
    memset(p_cccd_md, 0, sizeof(*p_cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&p_cccd_md->read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&p_cccd_md->write_perm);
    p_cccd_md->vloc = BLE_GATTS_VLOC_STACK;
}

static ret_code_t estc_ble_add_char(ble_estc_service_t *       service,
                                    uint16_t                   uuid,
                                    ble_gap_conn_sec_mode_t    read_perm,
                                    ble_gap_conn_sec_mode_t    write_perm,
                                    uint16_t                   val_len,
                                    uint8_t *                  p_init_val,
                                    ble_gatts_char_handles_t * p_handles)
{
    ble_gatts_attr_md_t cccd_md;
    cccd_md_init(&cccd_md);

    ble_gatts_char_md_t char_md = {0};
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1;
    char_md.p_cccd_md         = &cccd_md;

    ble_uuid_t char_uuid;
    char_uuid.type = service->uuid_type;
    char_uuid.uuid = uuid;

    ble_gatts_attr_md_t attr_md = {0};
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = read_perm;
    attr_md.write_perm = write_perm;

    ble_gatts_attr_t attr_char_value = {0};
    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = val_len;
    attr_char_value.max_len   = val_len;
    attr_char_value.p_value   = p_init_val;

    return sd_ble_gatts_characteristic_add(service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           p_handles);
}

static ret_code_t estc_ble_add_characteristics(ble_estc_service_t *service)
{
    ret_code_t error_code = NRF_SUCCESS;
    ble_gap_conn_sec_mode_t read_perm;
    ble_gap_conn_sec_mode_t write_perm;

    // Characteristic 1 - LED State
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&write_perm);
    uint8_t initial_state = 0;
    
    error_code = estc_ble_add_char(service,
                                   ESTC_GATT_CHAR_LED_STATE_UUID,
                                   read_perm, write_perm,
                                   sizeof(uint8_t), (uint8_t *)&initial_state,
                                   &service->led_state_handles);
    if (error_code != NRF_SUCCESS) return error_code;
    
    // Characteristic 2 - LED Color
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&write_perm);
    uint32_t initial_color = 0;

    error_code = estc_ble_add_char(service,
                                   ESTC_GATT_CHAR_LED_COLOR_UUID,
                                   read_perm, write_perm,
                                   sizeof(uint32_t), (uint8_t *)&initial_color,
                                   &service->led_color_handles);
    if (error_code != NRF_SUCCESS) return error_code;

    return NRF_SUCCESS;
}

void estc_ble_service_on_ble_event(const ble_evt_t *ble_evt, void *ctx)
{
    ble_estc_service_t *service = (ble_estc_service_t *)ctx;

    switch (ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            service->connection_handle = ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            service->connection_handle = BLE_CONN_HANDLE_INVALID;
            service->is_led_state_notifying = false;
            service->is_led_color_notifying = false;
            break;

        case BLE_GATTS_EVT_HVC:
            NRF_LOG_INFO("Indication confirmed by peer (handle 0x%04X).",
                         ble_evt->evt.gatts_evt.params.hvc.handle);
            break;

        case BLE_GATTS_EVT_WRITE:
        {
            ble_gatts_evt_write_t const * p_evt_write = &ble_evt->evt.gatts_evt.params.write;
            
            if (p_evt_write->handle == service->led_state_handles.cccd_handle && p_evt_write->len == 2)
            {
                service->is_led_state_notifying = ble_srv_is_notification_enabled(p_evt_write->data);
            }
            else if (p_evt_write->handle == service->led_color_handles.cccd_handle && p_evt_write->len == 2)
            {
                service->is_led_color_notifying = ble_srv_is_notification_enabled(p_evt_write->data);
            }
            else if (p_evt_write->handle == service->led_state_handles.value_handle 
                && p_evt_write->len == sizeof(uint8_t))
            {
                if (service->evt_handler != NULL)
                {
                    ble_estc_evt_t evt;
                    evt.evt_type = BLE_ESTC_EVT_LED_STATE_WRITE;
                    evt.params.led_state = p_evt_write->data[0];
                    service->evt_handler(service, &evt);
                }
            }
            else if (p_evt_write->handle == service->led_color_handles.value_handle 
                && p_evt_write->len == sizeof(uint32_t))
            {
                if (service->evt_handler != NULL)
                {
                    ble_estc_evt_t evt;
                    evt.evt_type = BLE_ESTC_EVT_LED_COLOR_WRITE;
                    memcpy(&evt.params.led_color, p_evt_write->data, sizeof(uint32_t));
                    service->evt_handler(service, &evt);
                }
            }
            break;
        }

        default:
            break;
    }
}


void estc_update_led_state(ble_estc_service_t *service, uint8_t state)
{
    if (service->connection_handle == BLE_CONN_HANDLE_INVALID)
    {
        return; 
    }

    ble_gatts_value_t gatts_value = {0};
    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &state;
    sd_ble_gatts_value_set(service->connection_handle, service->led_state_handles.value_handle, &gatts_value);

    if (service->is_led_state_notifying)
    {
        uint16_t len = sizeof(uint8_t);
        ble_gatts_hvx_params_t hvx_params = {0};
        hvx_params.handle = service->led_state_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = &state;

        sd_ble_gatts_hvx(service->connection_handle, &hvx_params);
    }
}

void estc_update_led_color(ble_estc_service_t *service, uint32_t color)
{
    if (service->connection_handle == BLE_CONN_HANDLE_INVALID)
    {
        return; 
    }

    ble_gatts_value_t gatts_value = {0};
    gatts_value.len     = sizeof(uint32_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&color;
    sd_ble_gatts_value_set(service->connection_handle, service->led_color_handles.value_handle, &gatts_value);

    if (service->is_led_color_notifying)
    {
        uint16_t len = sizeof(uint32_t);
        ble_gatts_hvx_params_t hvx_params = {0};
        hvx_params.handle = service->led_color_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t *)&color;

        sd_ble_gatts_hvx(service->connection_handle, &hvx_params);
    }
}