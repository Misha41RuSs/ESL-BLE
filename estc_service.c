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

    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = {ESTC_BASE_UUID};

    // TODO: 4. Add service UUIDs to the BLE stack table using `sd_ble_uuid_vs_add`
    error_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(error_code);
    service->uuid_type = service_uuid.type;

    service_uuid.uuid = ESTC_SERVICE_UUID;

    // TODO: 5. Add service to the BLE stack using `sd_ble_gatts_service_add`
    error_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                          &service_uuid,
                                          &service->service_handle);
    APP_ERROR_CHECK(error_code);

    return estc_ble_add_characteristics(service);
}

static ret_code_t estc_ble_add_characteristics(ble_estc_service_t *service)
{
    ret_code_t error_code = NRF_SUCCESS;

    ble_uuid_t char_uuid;

    // TODO: 6.1. Add custom characteristic UUID using `sd_ble_uuid_vs_add`, same as in step 4
    // Используем uuid_type сервиса (НЕ вызываем sd_ble_uuid_vs_add повторно)
    char_uuid.type = service->uuid_type;
    char_uuid.uuid = ESTC_GATT_CHAR_1_UUID;

    // TODO: 6.5. Configure Characteristic metadata (enable read and write)
    ble_gatts_char_md_t char_md = {0};
    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;

    // Configures attribute metadata. For now we only specify that the attribute will be stored in the softdevice
    ble_gatts_attr_md_t attr_md = {0};
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    // TODO: 6.6. Set read/write security levels to our attribute metadata using `BLE_GAP_CONN_SEC_MODE_SET_OPEN`
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    // TODO: 6.2. Configure the characteristic value attribute (set the UUID and metadata)
    ble_gatts_attr_t attr_char_value = {0};
    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;

    int32_t initial_value = 0;

    // TODO: 6.7. Set characteristic length in number of bytes in attr_char_value structure
    attr_char_value.init_len = sizeof(int32_t);
    attr_char_value.max_len  = sizeof(int32_t);
    attr_char_value.p_value  = (uint8_t *)&initial_value;

    // TODO: 6.4. Add new characteristic to the service using `sd_ble_gatts_characteristic_add`
    error_code = sd_ble_gatts_characteristic_add(service->service_handle,
                                                 &char_md,
                                                 &attr_char_value,
                                                 &service->char1_handles);
    APP_ERROR_CHECK(error_code);

    return NRF_SUCCESS;
}