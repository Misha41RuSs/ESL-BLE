#include "ble_core.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "smart_led.h"
#include "estc_service.h"

#define DEVICE_NAME                     "Mikhail Shvets"
#define APP_ADV_INTERVAL                300
#define APP_ADV_DURATION                18000
#define APP_BLE_OBSERVER_PRIO           3
#define APP_BLE_CONN_CFG_TAG            1

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)
#define SLAVE_LATENCY                   0
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define DEAD_BEEF                       0xDEADBEEF

NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
ble_estc_service_t m_estc_service;

static ble_uuid_t m_adv_uuids[] =
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    {ESTC_SERVICE_UUID, 0}
};

static void on_custom_ble_evt(ble_estc_service_t *service, ble_estc_evt_t *evt)
{
    switch (evt->evt_type)
    {
        case BLE_ESTC_EVT_LED_STATE_WRITE:
            smart_led_set_state(evt->params.led_state != 0);
            estc_update_led_state(&m_estc_service, evt->params.led_state, false);
            break;

        case BLE_ESTC_EVT_LED_COLOR_WRITE:
        {
            float h; int s, v;
            unpack_hsv(evt->params.led_color, &h, &s, &v);
            smart_led_set_color(h, s, v);
            estc_update_led_color(&m_estc_service, evt->params.led_color, false);
            break;
        }
    }
}

static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    m_estc_service.evt_handler = on_custom_ble_evt;
    err_code = estc_ble_service_init(&m_estc_service);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break;
        default:
            break;
    }
}

static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);
}

static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    sec_param.bond           = 1;
    sec_param.mitm           = 0;
    sec_param.lesc           = 0;
    sec_param.keypress       = 0;
    sec_param.io_caps        = BLE_GAP_IO_CAPS_NONE;
    sec_param.oob            = 0;
    sec_param.min_key_size   = 7;
    sec_param.max_key_size   = 16;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);
    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            estc_ble_service_on_ble_event(p_ble_evt, &m_estc_service);
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            estc_ble_service_on_ble_event(p_ble_evt, &m_estc_service);

            {
                estc_update_led_state(&m_estc_service, smart_led_get_state() ? 1 : 0, true);
                estc_update_led_color(&m_estc_service, smart_led_get_color_pack(), true);
            }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys = { .rx_phys = BLE_GAP_PHY_AUTO, .tx_phys = BLE_GAP_PHY_AUTO };
            sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            break;
        }

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        case BLE_GATTS_EVT_HVC:
            estc_ble_service_on_ble_event(p_ble_evt, &m_estc_service);
            break;

        case BLE_GATTS_EVT_WRITE:
            estc_ble_service_on_ble_event(p_ble_evt, &m_estc_service);
            break;

        default:
            break;
    }
}

static void ble_stack_init(void)
{
    ret_code_t err_code;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.flags     = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void on_led_state_changed(bool state)
{
    estc_update_led_state(&m_estc_service, state ? 1 : 0, true);
}

static void on_led_color_changed(uint32_t color_packed)
{
    estc_update_led_color(&m_estc_service, color_packed, true);
}

void ble_core_init(void)
{
    ble_stack_init();
    gap_params_init();
    gatt_init();
    peer_manager_init();
    services_init();
    smart_led_set_callbacks(on_led_state_changed, on_led_color_changed);
    m_adv_uuids[1].type = m_estc_service.uuid_type;
    advertising_init();
    conn_params_init();
}

void ble_core_advertising_start(void)
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}
