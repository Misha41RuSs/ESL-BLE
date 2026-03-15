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

    return estc_ble_add_characteristics(service);
}


static void cccd_md_init(ble_gatts_attr_md_t *p_cccd_md)
{
    memset(p_cccd_md, 0, sizeof(*p_cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&p_cccd_md->read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&p_cccd_md->write_perm);
    p_cccd_md->vloc = BLE_GATTS_VLOC_STACK;
}

static ret_code_t estc_ble_add_characteristics(ble_estc_service_t *service)
{
    ret_code_t error_code = NRF_SUCCESS;

    {
        ble_uuid_t char_uuid;
        char_uuid.type = service->uuid_type;
        char_uuid.uuid = ESTC_GATT_CHAR_1_UUID;

        ble_gatts_char_md_t char_md = {0};
        char_md.char_props.read  = 1;
        char_md.char_props.write = 1;

        ble_gatts_attr_md_t attr_md = {0};
        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        ble_gatts_attr_t attr_char_value = {0};
        attr_char_value.p_uuid    = &char_uuid;
        attr_char_value.p_attr_md = &attr_md;

        int32_t initial_value = 0;
        attr_char_value.init_len = sizeof(int32_t);
        attr_char_value.max_len  = sizeof(int32_t);
        attr_char_value.p_value  = (uint8_t *)&initial_value;

        error_code = sd_ble_gatts_characteristic_add(service->service_handle,
                                                     &char_md,
                                                     &attr_char_value,
                                                     &service->char1_handles);
        APP_ERROR_CHECK(error_code);
    }

    
    {
        ble_uuid_t char_uuid;
        char_uuid.type = service->uuid_type;
        char_uuid.uuid = ESTC_GATT_CHAR_2_UUID;

        ble_gatts_attr_md_t cccd_md;
        cccd_md_init(&cccd_md);

        ble_gatts_char_md_t char_md = {0};
        char_md.char_props.notify = 1;   
        char_md.p_cccd_md         = &cccd_md;

        ble_gatts_attr_md_t attr_md = {0};
        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        ble_gatts_attr_t attr_char_value = {0};
        attr_char_value.p_uuid    = &char_uuid;
        attr_char_value.p_attr_md = &attr_md;

        int32_t initial_value = 0;
        attr_char_value.init_len = sizeof(int32_t);
        attr_char_value.max_len  = sizeof(int32_t);
        attr_char_value.p_value  = (uint8_t *)&initial_value;

        error_code = sd_ble_gatts_characteristic_add(service->service_handle,
                                                     &char_md,
                                                     &attr_char_value,
                                                     &service->char2_handles);
        APP_ERROR_CHECK(error_code);
    }

    {
        ble_uuid_t char_uuid;
        char_uuid.type = service->uuid_type;
        char_uuid.uuid = ESTC_GATT_CHAR_3_UUID;

        ble_gatts_attr_md_t cccd_md;
        cccd_md_init(&cccd_md);

        ble_gatts_char_md_t char_md = {0};
        char_md.char_props.indicate = 1;  
        char_md.p_cccd_md           = &cccd_md;

        ble_gatts_attr_md_t attr_md = {0};
        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

        ble_gatts_attr_t attr_char_value = {0};
        attr_char_value.p_uuid    = &char_uuid;
        attr_char_value.p_attr_md = &attr_md;

        int32_t initial_value = 0;
        attr_char_value.init_len = sizeof(int32_t);
        attr_char_value.max_len  = sizeof(int32_t);
        attr_char_value.p_value  = (uint8_t *)&initial_value;

        error_code = sd_ble_gatts_characteristic_add(service->service_handle,
                                                     &char_md,
                                                     &attr_char_value,
                                                     &service->char3_handles);
        APP_ERROR_CHECK(error_code);
    }

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
            break;

        case BLE_GATTS_EVT_HVC:
            NRF_LOG_INFO("Indication confirmed by peer (handle 0x%04X).",
                         ble_evt->evt.gatts_evt.params.hvc.handle);
            break;

        default:
            break;
    }
}


void estc_update_characteristic_1_value(ble_estc_service_t *service, int32_t *value)
{
    if (service->connection_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }
    ble_gatts_value_t gatts_value = {0};
    gatts_value.len     = sizeof(int32_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *)value;

    ret_code_t err_code = sd_ble_gatts_value_set(service->connection_handle,
                                                  service->char1_handles.value_handle,
                                                  &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("char1 value set error: 0x%08X", err_code);
    }
}

void estc_update_characteristic_2_value(ble_estc_service_t *service, int32_t *value)
{
    if (service->connection_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    uint16_t len = sizeof(int32_t);

    ble_gatts_hvx_params_t hvx_params = {0};
    hvx_params.handle = service->char2_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &len;
    hvx_params.p_data = (uint8_t *)value;

    ret_code_t err_code = sd_ble_gatts_hvx(service->connection_handle, &hvx_params);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        NRF_LOG_ERROR("char2 notify error: 0x%08X", err_code);
    }
}

void estc_update_characteristic_3_value(ble_estc_service_t *service, int32_t *value)
{
    if (service->connection_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    uint16_t len = sizeof(int32_t);

    ble_gatts_hvx_params_t hvx_params = {0};
    hvx_params.handle = service->char3_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &len;
    hvx_params.p_data = (uint8_t *)value;

    ret_code_t err_code = sd_ble_gatts_hvx(service->connection_handle, &hvx_params);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        NRF_LOG_ERROR("char3 indicate error: 0x%08X", err_code);
    }
}