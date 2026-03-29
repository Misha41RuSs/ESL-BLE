#include "smart_led.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "nrfx_pwm.h"
#include "nrfx_gpiote.h"
#include "nrf_pwm.h"
#include "math.h"
#include "app_fds.h"
#include "estc_service.h"

extern ble_estc_service_t m_estc_service;

volatile led_ctx_t m_led_ctx = {
    .state = {0, 0.0f, 100, 100},
    .mode = MODE_NONE,
    .dir_h = 1,
    .dir_s = 1,
    .dir_v = 1,
    .indicator_duty = 0,
    .indicator_dir = 1,
    .indicator_step = 1,
    .indicator_period_ms = SLOW_BLINK_PERIOD_MS,
    .button_blocked = false,
    .first_click_detected = false,
    .button_held = false,
    .fds_file_id = 0x1111,
    .fds_rec_key = 0x3333
};

APP_TIMER_DEF(main_timer);
APP_TIMER_DEF(debounce_timer);
APP_TIMER_DEF(double_click_timer);
static nrfx_pwm_t m_pwm_instance = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t m_seq_values;

static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

uint32_t pack_hsv(float h, int s, int v) {
    return ((uint32_t)((int)h) << 16) | ((uint32_t)s << 8) | v;
}

void unpack_hsv(uint32_t packed, float *h, int *s, int *v) {
    *h = (float)((packed >> 16) & 0xFFFF);
    *s = (packed >> 8) & 0xFF;
    *v = packed & 0xFF;
    *h = clamp_int((int)*h, 0, 360);
    *s = clamp_int(*s, 0, 100);
    *v = clamp_int(*v, 0, 100);
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

uint16_t calculate_indicator(volatile led_ctx_t *ctx) {
    uint16_t ind = 0;
    if (ctx->mode == MODE_NONE) {
        ind = ctx->state.state ? 0 : 0;
        ctx->indicator_duty = 0;
    } else if (ctx->mode == MODE_VAL) {
        ind = PWM_TOP_VALUE;
        ctx->indicator_duty = PWM_TOP_VALUE;
    } else {
        if (ctx->indicator_period_ms > 0) {
            ctx->indicator_duty += (int)ctx->indicator_step * ctx->indicator_dir;
            if (ctx->indicator_duty >= (int)PWM_TOP_VALUE) {
                ctx->indicator_duty = PWM_TOP_VALUE;
                ctx->indicator_dir = -1;
            } else if (ctx->indicator_duty <= 0) {
                ctx->indicator_duty = 0;
                ctx->indicator_dir = 1;
            }
            ind = (uint16_t)clamp_int(ctx->indicator_duty, 0, PWM_TOP_VALUE);
        } else {
            ind = 0;
        }
    }
    return ind;
}

void apply_led_pwm(volatile led_ctx_t *ctx, uint16_t ind) {
    uint16_t r, g, b;
    bool is_on = ctx->state.state || ctx->mode != MODE_NONE;
    if (is_on) {
        hsv_to_rgb(ctx->state.h, ctx->state.s, ctx->state.v, &r, &g, &b);
    } else {
        r = 0; g = 0; b = 0;
    }
    pwm_write_channels(ind, r, g, b);
}

void smart_led_update_indicator_params_for_mode(volatile led_ctx_t *ctx) {
    app_timer_stop(main_timer);
    switch (ctx->mode) {
        case MODE_NONE:
            ctx->indicator_period_ms = 0;
            ctx->indicator_duty = 0;
            ctx->indicator_dir = 1;
            break;
        case MODE_HUE:
            ctx->indicator_period_ms = SLOW_BLINK_PERIOD_MS;
            app_timer_start(main_timer, APP_TIMER_TICKS(MAIN_INTERVAL_MS), NULL);
            break;
        case MODE_SAT:
            ctx->indicator_period_ms = FAST_BLINK_PERIOD_MS;
            app_timer_start(main_timer, APP_TIMER_TICKS(MAIN_INTERVAL_MS), NULL);
            break;
        case MODE_VAL:
            ctx->indicator_period_ms = 1;
            ctx->indicator_duty = PWM_TOP_VALUE;
            app_timer_start(main_timer, APP_TIMER_TICKS(MAIN_INTERVAL_MS), NULL);
            break;
    }
    if (ctx->indicator_period_ms > 0) {
        ctx->indicator_step = (int)ceilf((float)PWM_TOP_VALUE * ((float)MAIN_INTERVAL_MS / (ctx->indicator_period_ms / 2.0f)));
        if (ctx->indicator_step < 1) ctx->indicator_step = 1;
    } else {
        ctx->indicator_step = PWM_TOP_VALUE;
    }
    apply_led_pwm(ctx, calculate_indicator(ctx));
}

static void process_smooth_transition(volatile led_ctx_t *ctx) {
    if (ctx->button_held && ctx->mode != MODE_NONE) {
        if (ctx->mode == MODE_HUE) {
            ctx->state.h += ctx->dir_h * HOLD_STEP_H;
            if (ctx->state.h >= 360.0f) { ctx->state.h = 360.0f; ctx->dir_h = -1; } 
            else if (ctx->state.h <= 0.0f) { ctx->state.h = 0.0f; ctx->dir_h = 1; }
        }
        else if (ctx->mode == MODE_SAT) {
            ctx->state.s += ctx->dir_s * HOLD_STEP_SV;
            if (ctx->state.s >= 100) { ctx->state.s = 100; ctx->dir_s = -1; } 
            else if (ctx->state.s <= 0) { ctx->state.s = 0; ctx->dir_s = 1; }
        }
        else if (ctx->mode == MODE_VAL) {
            ctx->state.v += ctx->dir_v * HOLD_STEP_SV;
            if (ctx->state.v >= 100) { ctx->state.v = 100; ctx->dir_v = -1; } 
            else if (ctx->state.v <= 0) { ctx->state.v = 0; ctx->dir_v = 1; }
        }
    }
}


void main_timer_handler(void *p_context) {
    (void)p_context;
    volatile led_ctx_t *ctx = &m_led_ctx;
    process_smooth_transition(ctx);
    uint16_t ind = calculate_indicator(ctx);
    apply_led_pwm(ctx, ind);
}

void debounce_timer_handler(void *p_context) {
    (void)p_context;
    volatile led_ctx_t *ctx = &m_led_ctx;
    bool is_pressed = nrf_gpio_pin_read(BUTTON_PIN) == 0; 
    if (is_pressed) {
        ctx->button_held = true;
        if (ctx->first_click_detected) {
            ctx->first_click_detected = false;
            app_timer_stop(double_click_timer);
            input_mode_t old_mode = ctx->mode;
            if (ctx->mode == MODE_NONE) ctx->mode = MODE_HUE;
            else if (ctx->mode == MODE_HUE) ctx->mode = MODE_SAT;
            else if (ctx->mode == MODE_SAT) ctx->mode = MODE_VAL;
            else ctx->mode = MODE_NONE;
            
            if (ctx->mode == MODE_NONE && old_mode != MODE_NONE) {
                ctx->state.state = 1;
                app_fds_save_config(ctx);
                estc_update_led_color(&m_estc_service, pack_hsv(ctx->state.h, ctx->state.s, ctx->state.v), true);
                estc_update_led_state(&m_estc_service, ctx->state.state, true);
            }
            ctx->dir_h = 1; ctx->dir_s = 1; ctx->dir_v = 1;
            smart_led_update_indicator_params_for_mode(ctx);
        } else {
            ctx->first_click_detected = true;
            app_timer_start(double_click_timer, APP_TIMER_TICKS(DOUBLE_CLICK_MS), NULL);
        }
    } else {
        ctx->button_held = false;
    }
    ctx->button_blocked = false;
}

void double_click_timer_handler(void *p_context) {
    (void)p_context;
    volatile led_ctx_t *ctx = &m_led_ctx;
    if (ctx->mode == MODE_NONE && !ctx->button_held) {
        ctx->state.state = ctx->state.state ? 0 : 1;
        app_fds_save_config(ctx);
        
        estc_update_led_state(&m_estc_service, ctx->state.state, true);
    }
    ctx->first_click_detected = false;
    apply_led_pwm(ctx, calculate_indicator(ctx));
}

void button_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    (void)pin; (void)action;
    volatile led_ctx_t *ctx = &m_led_ctx;
    if (ctx->button_blocked) return;
    ctx->button_blocked = true;
    app_timer_start(debounce_timer, APP_TIMER_TICKS(DEBOUNCE_MS), NULL);
}

static void pwm_init(void) {
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
}

static void button_init(void) {
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

void smart_led_init(void) {
    pwm_init();
    button_init();
    smart_led_update_indicator_params_for_mode(&m_led_ctx);
}
