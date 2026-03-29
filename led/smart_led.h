#ifndef SMART_LED_H__
#define SMART_LED_H__

#include <stdint.h>
#include <stdbool.h>

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

typedef struct {
    uint8_t state;
    float h;
    int s;
    int v;
} led_state_t;

typedef struct {
    uint8_t state;
    uint32_t color;
} led_fds_config_t;

typedef struct {
    led_state_t state;
    input_mode_t mode;
    int dir_h;
    int dir_s;
    int dir_v;

    int indicator_duty;
    int indicator_dir;
    uint32_t indicator_step;
    uint32_t indicator_period_ms;

    bool button_blocked;
    bool first_click_detected;
    bool button_held;

    led_fds_config_t fds_config;
    uint16_t fds_file_id;
    uint16_t fds_rec_key;
} led_ctx_t;

extern volatile led_ctx_t m_led_ctx;

typedef void (*smart_led_state_cb_t)(bool state);
typedef void (*smart_led_color_cb_t)(uint32_t color_packed);

void smart_led_init(void);
void smart_led_set_callbacks(smart_led_state_cb_t state_cb, smart_led_color_cb_t color_cb);

void smart_led_set_color(float h, int s, int v);
void smart_led_set_state(bool state);
uint32_t smart_led_get_color_pack(void);
bool smart_led_get_state(void);

uint32_t pack_hsv(float h, int s, int v);
void unpack_hsv(uint32_t packed, float *h, int *s, int *v);

#endif // SMART_LED_H__
