#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_usb.h"

#include "smart_led.h"
#include "app_fds.h"
#include "ble_core.h"

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

#define DEAD_BEEF 0xDEADBEEF
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

int main(void)
{
    log_init();
    app_timer_init();
    power_management_init();

    ble_core_init();
    app_fds_init();
    smart_led_init();

    NRF_LOG_INFO("ESTC advertising LED Control service started.");
    ble_core_advertising_start();

    for (;;)
    {
        idle_state_handle();
    }
}
