#ifndef APP_FDS_H__
#define APP_FDS_H__

#include <stdint.h>
#include <stdbool.h>
#include "smart_led.h"

void app_fds_init(void);
void app_fds_save_config(volatile led_ctx_t *ctx);

#endif // APP_FDS_H__
