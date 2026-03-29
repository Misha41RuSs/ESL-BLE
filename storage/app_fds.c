#include "app_fds.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "app_error.h"
#include <string.h>

static bool m_fds_initialized = false;

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->id == FDS_EVT_INIT) {
        if (p_evt->result == NRF_SUCCESS) {
            m_fds_initialized = true;
        }
    }
}

void app_fds_save_config(volatile led_ctx_t *ctx)
{
    fds_record_t        record;
    fds_record_desc_t   record_desc;
    fds_find_token_t    ftok = {0};

    ctx->fds_config.state = ctx->state.state;
    ctx->fds_config.color = pack_hsv(ctx->state.h, ctx->state.s, ctx->state.v);

    record.file_id           = ctx->fds_file_id;
    record.key               = ctx->fds_rec_key;
    record.data.p_data       = (void*)&ctx->fds_config;
    record.data.length_words = (sizeof(led_fds_config_t) + 3) / 4;

    if (fds_record_find(ctx->fds_file_id, ctx->fds_rec_key, &record_desc, &ftok) == NRF_SUCCESS)
    {
        fds_record_update(&record_desc, &record);
    }
    else
    {
        fds_record_write(&record_desc, &record);
    }
}

void app_fds_init(void)
{
    volatile led_ctx_t *ctx = &m_led_ctx;
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

    if (fds_record_find(ctx->fds_file_id, ctx->fds_rec_key, &desc, &ftok) == NRF_SUCCESS)
    {
        fds_flash_record_t config;
        if (fds_record_open(&desc, &config) == NRF_SUCCESS)
        {
            memcpy((void*)&ctx->fds_config, config.p_data, sizeof(led_fds_config_t));
            fds_record_close(&desc);
            ctx->state.state = ctx->fds_config.state;
            unpack_hsv(ctx->fds_config.color, (float*)&ctx->state.h, (int*)&ctx->state.s, (int*)&ctx->state.v);
        }
    }
    else
    {
        ctx->state.state = 1;
        ctx->state.s = 100; ctx->state.v = 100;
        // DEVICE_ID = 6577 => Last digits = 77
        // Hue = 77% of 360 = 277.2 degrees
        ctx->state.h = (77.0f / 100.0f) * 360.0f;
    }
}
