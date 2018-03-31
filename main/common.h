#ifndef _COMMON_H_
#define _COMMON_H_
#include "freertos/FreeRTOS.h"

void send_keyboard_value(uint8_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key);
void update_config();

void app_main_bt();
void app_main_gpio();
void app_main_wifi();

#endif /* _COMMON_H_ */
