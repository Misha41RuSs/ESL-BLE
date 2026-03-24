/** @file
 *
 * @defgroup estc_adverts main.c
 * @{
 * @ingroup estc_templates
 * @brief ESTC Advertisments template app.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

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
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_gpio.h"
#include "nrfx_pwm.h"
#include "nrfx_gpiote.h"
#include "nrf_pwm.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_usb.h"
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
static ble_estc_service_t m_estc_service;

static ble_uuid_t m_adv_uuids[] =
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    {ESTC_SERVICE_UUID, 0}
};

#define LED0_PIN 6
#define LED1_PIN 8
#define LED2_PIN 41
#define LED3_PIN 12
#define BUTTON_PIN 38
#define PWM_CHANNELS 4
#define PWM_TOP_VALUE 1000U
#define MAIN_INTERVAL_MS 20
#define DEBOUNCE_MS 50
#define DOUBLE_CLICK_MS 400
#define HOLD_INTERVAL_MS MAIN_INTERVAL_MS
#define HOLD_STEP_H 1
#define HOLD_STEP_SV 1
#define SLOW_BLINK_PERIOD_MS 1500
#define FAST_BLINK_PERIOD_MS 500

typedef enum {
    MODE_NONE = 0,
    MODE_HUE,
    MODE_SAT,
    MODE_VAL
} input_mode_t;

volatile input_mode_t m_mode = MODE_NONE;
volatile float m_h = 0.0f;
volatile int m_s = 100;
volatile int m_v = 100;
volatile int dir_h = 1;
volatile int dir_s = 1;
volatile int dir_v = 1;
volatile int m_indicator_duty = 0;
volatile int m_indicator_dir = 1;
volatile bool m_button_blocked = false;
volatile bool m_first_click_detected = false;
volatile bool m_button_held = false;

APP_TIMER_DEF(main_timer);
APP_TIMER_DEF(debounce_timer);
APP_TIMER_DEF(double_click_timer);
static uint32_t m_indicator_step = 1;
static uint32_t m_indicator_period_ms = SLOW_BLINK_PERIOD_MS;
static nrfx_pwm_t m_pwm_instance = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t m_seq_values;

// FDS definitions
#define FDS_FILE_ID 0x1111
#define FDS_REC_KEY 0x3333 
static bool m_fds_initialized = false;
static uint8_t m_led_on = 0; 
typedef struct {
    uint8_t state;
    uint32_t color;
} led_config_t;
static led_config_t m_led_config;

static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void update_indicator_params_for_mode(void) {
    switch (m_mode) {
        case MODE_NONE:
            m_indicator_period_ms = 0;
            m_indicator_duty = 0;
            m_indicator_dir = 1;
            break;
        case MODE_HUE:
            m_indicator_period_ms = SLOW_BLINK_PERIOD_MS;
            break;
        case MODE_SAT:
            m_indicator_period_ms = FAST_BLINK_PERIOD_MS;
            break;
        case MODE_VAL:
            m_indicator_period_ms = 1;
            m_indicator_duty = PWM_TOP_VALUE;
            break;
    }
    if (m_indicator_period_ms > 0) {
        m_indicator_step = (int)ceilf((float)PWM_TOP_VALUE * ((float)MAIN_INTERVAL_MS / (m_indicator_period_ms / 2.0f)));
        if (m_indicator_step < 1) m_indicator_step = 1;
    } else {
        m_indicator_step = PWM_TOP_VALUE;
    }
}

static uint32_t pack_hsv(void) {
    return ((uint32_t)((int)m_h) << 16) | ((uint32_t)m_s << 8) | m_v;
}

static void unpack_hsv(uint32_t packed) {
    m_h = (float)((packed >> 16) & 0xFFFF);
    m_s = (packed >> 8) & 0xFF;
    m_v = packed & 0xFF;
    m_h = clamp_int((int)m_h, 0, 360);
    m_s = clamp_int(m_s, 0, 100);
    m_v = clamp_int(m_v, 0, 100);
}

static void fds_save_config(void)
{
    fds_record_t        record;
    fds_record_desc_t   record_desc;
    fds_find_token_t    ftok = {0};

    m_led_config.state = m_led_on;
    m_led_config.color = pack_hsv();

    record.file_id           = FDS_FILE_ID;
    record.key               = FDS_REC_KEY;
    record.data.p_data       = &m_led_config;
    record.data.length_words = (sizeof(led_config_t) + 3) / 4;

    if (fds_record_find(FDS_FILE_ID, FDS_REC_KEY, &record_desc, &ftok) == NRF_SUCCESS)
    {
        fds_record_update(&record_desc, &record);
    }
    else
    {
        fds_record_write(&record_desc, &record);
    }
}

void hsv_to_rgb(float h, int s, int v, uint16_t *r, uint16_t *g, uint16_t *b) {
    float H = h;
    float S = s / 100.0f;
    float V = v / 100.0f;

    if (S <= 0.0f) {
        uint16_t val = (uint16_t)(V * PWM_TOP_VALUE + 0.5f);
        *r = *g = *b = val;
        return;
    }

    if (H >= 360.0f) H = 0.0f;
    float hf = H / 60.0f;
    int i = (int)floorf(hf);
    float f = hf - i;
    float p = V * (1.0f - S);
    float q = V * (1.0f - S * f);
    float t = V * (1.0f - S * (1.0f - f));

    float rf=0, gf=0, bf=0;
    switch (i) {
        case 0: rf = V; gf = t; bf = p; break;
        case 1: rf = q; gf = V; bf = p; break;
        case 2: rf = p; gf = V; bf = t; break;
        case 3: rf = p; gf = q; bf = V; break;
        case 4: rf = t; gf = p; bf = V; break;
        case 5:
        default: rf = V; gf = p; bf = q; break;
    }

    *r = (uint16_t)(clamp_int((int)roundf(rf * PWM_TOP_VALUE), 0, PWM_TOP_VALUE));
    *g = (uint16_t)(clamp_int((int)roundf(gf * PWM_TOP_VALUE), 0, PWM_TOP_VALUE));
    *b = (uint16_t)(clamp_int((int)roundf(bf * PWM_TOP_VALUE), 0, PWM_TOP_VALUE));
}

static void pwm_write_channels(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3) {
    m_seq_values.channel_0 = ch0;
    m_seq_values.channel_1 = ch1;
    m_seq_values.channel_2 = ch2;
    m_seq_values.channel_3 = ch3;
}

void main_timer_handler(void *p_context) {
    (void)p_context;
    if (m_button_held && m_mode != MODE_NONE) {
        if (m_mode == MODE_HUE) {
            m_h += dir_h * HOLD_STEP_H;
            if (m_h >= 360.0f) { m_h = 360.0f; dir_h = -1; } 
            else if (m_h <= 0.0f) { m_h = 0.0f; dir_h = 1; }
        }
        else if (m_mode == MODE_SAT) {
            m_s += dir_s * HOLD_STEP_SV;
            if (m_s >= 100) { m_s = 100; dir_s = -1; } 
            else if (m_s <= 0) { m_s = 0; dir_s = 1; }
        }
        else if (m_mode == MODE_VAL) {
            m_v += dir_v * HOLD_STEP_SV;
            if (m_v >= 100) { m_v = 100; dir_v = -1; } 
            else if (m_v <= 0) { m_v = 0; dir_v = 1; }
        }
    }

    uint16_t ind = 0;
    if (m_mode == MODE_NONE) {
        ind = m_led_on ? 0 : 0;
        m_indicator_duty = 0;
    } else if (m_mode == MODE_VAL) {
        ind = PWM_TOP_VALUE;
        m_indicator_duty = PWM_TOP_VALUE;
    } else {
        if (m_indicator_period_ms > 0) {
            m_indicator_duty += (int)m_indicator_step * m_indicator_dir;
            if (m_indicator_duty >= (int)PWM_TOP_VALUE) {
                m_indicator_duty = PWM_TOP_VALUE;
                m_indicator_dir = -1;
            } else if (m_indicator_duty <= 0) {
                m_indicator_duty = 0;
                m_indicator_dir = 1;
            }
            ind = (uint16_t)clamp_int(m_indicator_duty, 0, PWM_TOP_VALUE);
        } else {
            ind = 0;
        }
    }

    uint16_t r, g, b;
    if (m_led_on || m_mode != MODE_NONE) {
        hsv_to_rgb(m_h, m_s, m_v, &r, &g, &b);
    } else {
        r = 0;
        g = 0; 
        b = 0;
    }

    pwm_write_channels(ind, r, g, b);
}

void debounce_timer_handler(void *p_context) {
    (void)p_context;
    bool is_pressed = nrf_gpio_pin_read(BUTTON_PIN) == 0; 
    if (is_pressed) {
        m_button_held = true;
        if (m_first_click_detected) {
            m_first_click_detected = false;
            app_timer_stop(double_click_timer);
            input_mode_t old_mode = m_mode;
            if (m_mode == MODE_NONE) m_mode = MODE_HUE;
            else if (m_mode == MODE_HUE) m_mode = MODE_SAT;
            else if (m_mode == MODE_SAT) m_mode = MODE_VAL;
            else m_mode = MODE_NONE;
            
            if (m_mode == MODE_NONE && old_mode != MODE_NONE) {
                m_led_on = 1;
                fds_save_config();
                estc_update_led_color(&m_estc_service, pack_hsv());
                estc_update_led_state(&m_estc_service, m_led_on);
            }
            dir_h = 1; dir_s = 1; dir_v = 1;
            update_indicator_params_for_mode();
        } else {
            m_first_click_detected = true;
            app_timer_start(double_click_timer, APP_TIMER_TICKS(DOUBLE_CLICK_MS), NULL);
        }
    } else {
        m_button_held = false;
    }
    m_button_blocked = false;
}

void double_click_timer_handler(void *p_context) {
    (void)p_context;
    if (m_mode == MODE_NONE && !m_button_held) {
        m_led_on = m_led_on ? 0 : 1;
        fds_save_config();
        
        estc_update_led_state(&m_estc_service, m_led_on);
    }
    m_first_click_detected = false;
}

void button_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    (void)pin; (void)action;
    if (m_button_blocked) return;
    m_button_blocked = true;
    app_timer_start(debounce_timer, APP_TIMER_TICKS(DEBOUNCE_MS), NULL);
}

void pwm_init(void) {
    nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG;
    config.output_pins[0] = LED0_PIN;   
    config.output_pins[1] = LED1_PIN;   
    config.output_pins[2] = LED2_PIN;   
    config.output_pins[3] = LED3_PIN;   
    config.base_clock = NRF_PWM_CLK_1MHz;
    config.count_mode = NRF_PWM_MODE_UP;
    config.top_value  = PWM_TOP_VALUE;
    config.load_mode  = NRF_PWM_LOAD_INDIVIDUAL;
    config.step_mode  = NRF_PWM_STEP_AUTO;

    ret_code_t err_code = nrfx_pwm_init(&m_pwm_instance, &config, NULL);
    APP_ERROR_CHECK(err_code);

    m_seq_values.channel_0 = 0; m_seq_values.channel_1 = 0;
    m_seq_values.channel_2 = 0; m_seq_values.channel_3 = 0;

    nrf_pwm_sequence_t seq = {
        .values.p_individual = &m_seq_values,
        .length = PWM_CHANNELS,
        .repeats = 0,
        .end_delay = 0
    };

    nrfx_pwm_simple_playback(&m_pwm_instance, &seq, 1, NRFX_PWM_FLAG_LOOP);

    err_code = app_timer_create(&main_timer, APP_TIMER_MODE_REPEATED, main_timer_handler);
    APP_ERROR_CHECK(err_code);
    app_timer_start(main_timer, APP_TIMER_TICKS(MAIN_INTERVAL_MS), NULL);
}

void button_init(void) {
    if (!nrfx_gpiote_is_init()) {
        nrfx_gpiote_init();
    }
    nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
    nrfx_gpiote_in_config_t in_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_cfg.pull = NRF_GPIO_PIN_PULLUP;
    nrfx_gpiote_in_init(BUTTON_PIN, &in_cfg, button_handler);
    nrfx_gpiote_in_event_enable(BUTTON_PIN, true);

    app_timer_create(&debounce_timer, APP_TIMER_MODE_SINGLE_SHOT, debounce_timer_handler);
    app_timer_create(&double_click_timer, APP_TIMER_MODE_SINGLE_SHOT, double_click_timer_handler);
}

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->id == FDS_EVT_INIT) {
        if (p_evt->result == NRF_SUCCESS) {
            m_fds_initialized = true;
        }
    }
}

// BLE Event Callbacks
static void on_custom_ble_evt(ble_estc_service_t *service, ble_estc_evt_t *evt)
{
    switch (evt->evt_type)
    {
        case BLE_ESTC_EVT_LED_STATE_WRITE:
            m_led_on = (evt->params.led_state == 0 || evt->params.led_state == '0') ? 0 : 1;
            fds_save_config();
            estc_update_led_state(&m_estc_service, m_led_on);
            break;

        case BLE_ESTC_EVT_LED_COLOR_WRITE:
            unpack_hsv(evt->params.led_color);
            fds_save_config();
            estc_update_led_color(&m_estc_service, evt->params.led_color);
            break;
    }
}

static void advertising_start(void);

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void fds_manager_init(void)
{
    ret_code_t rc;
    rc = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(rc);
    rc = fds_init();
    APP_ERROR_CHECK(rc);

    while(!m_fds_initialized) {
        nrf_pwr_mgmt_run();
    }

    fds_record_desc_t  desc = {0};
    fds_find_token_t   ftok = {0};

    if (fds_record_find(FDS_FILE_ID, FDS_REC_KEY, &desc, &ftok) == NRF_SUCCESS)
    {
        fds_flash_record_t config;
        if (fds_record_open(&desc, &config) == NRF_SUCCESS)
        {
            memcpy(&m_led_config, config.p_data, sizeof(led_config_t));
            fds_record_close(&desc);
            m_led_on = m_led_config.state;
            unpack_hsv(m_led_config.color);
        }
    }
    else
    {
        m_led_on = 1;
        m_s = 100; m_v = 100;
        // DEVICE_ID = 6577 => Last digits = 77
        // Hue = 77% of 360 = 277.2 degrees

        m_h = (77.0f / 100.0f) * 360.0f;
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

            estc_update_led_state(&m_estc_service, m_led_on);
            estc_update_led_color(&m_estc_service, pack_hsv());
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

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
    LOG_BACKEND_USB_PROCESS();
}

static void advertising_start(void)
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    log_init();
    app_timer_init();
    power_management_init();

    ble_stack_init();
    gap_params_init();
    gatt_init();
    peer_manager_init();
    
    fds_manager_init();

    services_init();
    m_adv_uuids[1].type = m_estc_service.uuid_type;
    advertising_init();
    conn_params_init();

    pwm_init();
    button_init();
    
    update_indicator_params_for_mode();

    NRF_LOG_INFO("ESTC advertising LED Control service started.");
    advertising_start();

    for (;;)
    {
        idle_state_handle();
    }
}
