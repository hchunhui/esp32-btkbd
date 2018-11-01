#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "keylayouts.h"
#include "common.h"

#define GPIO_OUTPUT_IO_0    22
#define GPIO_OUTPUT_IO_1    5
#define GPIO_OUTPUT_IO_2    23
#define GPIO_OUTPUT_IO_3    16
#define GPIO_OUTPUT_IO_4    18
#define GPIO_OUTPUT_IO_5    21
#define GPIO_OUTPUT_IO_6    19
#define GPIO_OUTPUT_IO_7    17


#define GPIO_OUTPUT_PIN_SEL  ( \
		(1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | \
		(1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3) |	\
		(1ULL<<GPIO_OUTPUT_IO_4) | (1ULL<<GPIO_OUTPUT_IO_5) |	\
		(1ULL<<GPIO_OUTPUT_IO_6) | (1ULL<<GPIO_OUTPUT_IO_7))

#define GPIO_INPUT_IO_0     3
#define GPIO_INPUT_IO_1     25
#define GPIO_INPUT_IO_2     10
#define GPIO_INPUT_IO_3     14
#define GPIO_INPUT_IO_4     33
#define GPIO_INPUT_IO_5     1
#define GPIO_INPUT_IO_6     32
#define GPIO_INPUT_IO_7     26


#define GPIO_INPUT_PIN_SEL  ( \
	(1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) |		\
	(1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3) |		\
	(1ULL<<GPIO_INPUT_IO_4) | (1ULL<<GPIO_INPUT_IO_5) |		\
	(1ULL<<GPIO_INPUT_IO_6) | (1ULL<<GPIO_INPUT_IO_7))

#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static const char *TAG = "gpiokey";

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static int io2row[] = {GPIO_OUTPUT_IO_0, GPIO_OUTPUT_IO_1,
		       GPIO_OUTPUT_IO_2, GPIO_OUTPUT_IO_3,
		       GPIO_OUTPUT_IO_4, GPIO_OUTPUT_IO_5,
		       GPIO_OUTPUT_IO_6, GPIO_OUTPUT_IO_7};
static int io2col[] = {GPIO_INPUT_IO_0, GPIO_INPUT_IO_1,
		       GPIO_INPUT_IO_2, GPIO_INPUT_IO_3,
		       GPIO_INPUT_IO_4, GPIO_INPUT_IO_5,
		       GPIO_INPUT_IO_6, GPIO_INPUT_IO_7};
#define IOX 8
#define IOY 8
static int keymap[IOX][IOY];
static int map[2][IOX][IOY] = {
	{
	{KEY_TILDE, KEY_1, KEY_3, KEY_5, KEY_7, KEY_9, KEY_MINUS, KEY_RIGHT, },
	{KEY_TAB, KEY_2, KEY_4, KEY_6, KEY_8, KEY_0, KEY_EQUAL, KEY_UP, },
	{KEY_CAPS_LOCK, KEY_Q, KEY_E, KEY_T, KEY_U, KEY_O, KEY_LEFT_BRACE, KEY_DOWN, },
	{KEY_LEFT_SHIFT, KEY_W, KEY_R, KEY_Y, KEY_I, KEY_P, KEY_RIGHT_BRACE, KEY_LEFT, },
	{KEY_LEFT_CTRL, KEY_A, KEY_D, KEY_G, KEY_J, KEY_L, KEY_QUOTE, KEY_RIGHT_SHIFT, },
	{KEY_LEFT_GUI, KEY_S, KEY_F, KEY_H, KEY_K, KEY_SEMICOLON, KEY_ENTER, KEY_RIGHT_CTRL, },
	{KEY_LEFT_ALT, KEY_Z, KEY_C, KEY_B, KEY_M, KEY_PERIOD, KEY_BACKSPACE, 0, },
	{KEY_SPACE, KEY_X, KEY_V, KEY_N, KEY_COMMA, KEY_SLASH, KEY_BACKSLASH, KEY_RIGHT_ALT, },
	},
	{
	{KEY_ESC, KEY_F1, KEY_F3, KEY_F5, KEY_F7, KEY_F9, KEY_F11, KEY_END, },
	{KEY_TAB, KEY_F2, KEY_F4, KEY_F6, KEY_F8, KEY_F10, KEY_F12, KEY_PAGE_UP, },
	{KEY_CAPS_LOCK, KEY_Q, 0x9902, KEY_T, KEY_U, KEY_O, KEY_LEFT_BRACE, KEY_PAGE_DOWN, },
	{KEY_LEFT_SHIFT, KEY_W, KEY_R, KEY_Y, KEY_I, KEY_P, KEY_RIGHT_BRACE, KEY_HOME, },
	{KEY_LEFT_CTRL, KEY_A, 0x9904, KEY_G, 0x9905, 0x9907, KEY_QUOTE, KEY_RIGHT_SHIFT, },
	{KEY_LEFT_GUI, 0x9901, 0x9903, KEY_H, 0x9906, KEY_SEMICOLON, KEY_ENTER, KEY_RIGHT_CTRL, },
	{KEY_LEFT_ALT, KEY_Z, KEY_C, KEY_B, KEY_M, KEY_PERIOD, KEY_DELETE, 0, },
	{KEY_SPACE, KEY_X, KEY_V, KEY_N, KEY_COMMA, 0x99ff, KEY_INSERT, KEY_RIGHT_ALT, },
	},
};
static int midx;

static TickType_t caps_time;
static uint8_t keycode_modifier;
static uint8_t keycode_arr[6];

static uint8_t remove_keycode(uint8_t keycode,uint8_t *keycode_arr)
{
	uint8_t ret = 1;
	if(keycode == 0) return 1;
	//source: Arduino core Keyboard.cpp
	for (uint8_t i=0; i<6; i++) {
		if (keycode_arr[i] == keycode)
		{
			keycode_arr[i] = 0;
			ret = 0;
		}
	}
	return ret;
}

static uint8_t add_keycode(uint8_t keycode,uint8_t *keycode_arr)
{
	uint8_t i;
	//source: Arduino core Keyboard.cpp
	// Add k to the key array only if it's not already present
	// and if there is an empty slot.
	if (keycode_arr[0] != keycode && keycode_arr[1] != keycode &&
	    keycode_arr[2] != keycode && keycode_arr[3] != keycode &&
	    keycode_arr[4] != keycode && keycode_arr[5] != keycode) {

		for (i=0; i<6; i++) {
			if (keycode_arr[i] == 0x00) {
				keycode_arr[i] = keycode;
				return 0;
			}
		}
		if (i == 6) {
			return 2;
		}
	}
	return 1;
}

static void clear_keycode(uint8_t *keycode_arr)
{
	for (int i = 0; i < 6; i++)
		keycode_arr[i] = 0;
}

static int check(int x, int y)
{
	int i, j;

	for (i = 0; i < IOX; i++)
		if (i != x)
			for (j = 0; j < IOY; j++)
				if (j != y)
					if (keymap[i][j] +
					    keymap[i][y] +
					    keymap[x][j] >= 2)
						return 0;
	return 1;
}

static void xsend(int code)
{
	uint8_t keycode = 0;
	keycode = code;
	send_keyboard_value(0, &keycode, 1);
	keycode = 0;
	send_keyboard_value(0, &keycode, 1);
}

static void xsend_string(const unsigned char *s)
{
	uint8_t keycode = 0;
	int i;
	for (i = 0; s[i]; i++) {
		if(isalpha(s[i])) {
			if (isupper(s[i])) {
				keycode = s[i] - 'A' + KEY_A;
				send_keyboard_value(KEY_LEFT_SHIFT, &keycode, 1);
			} else {
				keycode = s[i] - 'a' + KEY_A;
				send_keyboard_value(0, &keycode, 1);
			}
		} else if (isdigit(s[i])) {
			if (s[i] == '0')
				keycode = KEY_0;
			else
				keycode = s[i] - '1' + KEY_1;
			send_keyboard_value(0, &keycode, 1);
		} else if (s[i] == '&') {
			keycode = KEY_7;
			send_keyboard_value(KEY_LEFT_SHIFT, &keycode, 1);
		} else if (s[i] == '_') {
			keycode = KEY_MINUS;
			send_keyboard_value(KEY_LEFT_SHIFT, &keycode, 1);
		}

		keycode = 0;
		send_keyboard_value(0, &keycode, 1);
	}
}

static void on_keydown(int i, int j)
{
	ESP_LOGI(TAG, "key down %d %d", i, j);
	int type = map[midx][i][j] >> 8;
	if (type == 0xf0) {
		if (map[midx][i][j] == KEY_CAPS_LOCK) {
			TickType_t now = xTaskGetTickCount();
			if (now - caps_time < pdMS_TO_TICKS(500)) {
				xsend(KEY_CAPS_LOCK);
				add_keycode(KEY_ESC & 0xff, keycode_arr);
				caps_time = 0;
			} else {
				add_keycode(KEY_CAPS_LOCK & 0xff, keycode_arr);
				caps_time = now;
			}
		} else {
			add_keycode(map[midx][i][j], keycode_arr);
		}
	} else if (type == 0xe0)
		keycode_modifier |= map[midx][i][j] & 0xff;
	else if (type == 0x99) {
		switch(map[midx][i][j] & 0xff) {
		case 1:
			send_mouse_value(0,-20,0,0); break;
		case 2:
			send_mouse_value(0,0,-20,0); break;
		case 3:
			send_mouse_value(0,20,0,0); break;
		case 4:
			send_mouse_value(0,0,20,0); break;
		case 5:
			send_mouse_value(0x01,0,0,0); break;
		case 6:
			send_mouse_value(0x04,0,0,0); break;
		case 7:
			send_mouse_value(0x02,0,0,0); break;
		case 0xff:
			xsend_string((void *)"foobar"); break;
		}
	} else {
		if (midx == 0) {
			midx = 1;
			keycode_modifier = 0;
		}
	}
}

static void on_keyup(int i, int j)
{
	ESP_LOGI(TAG, "key up %d %d", i, j);
	int type = map[midx][i][j] >> 8;
	if (type == 0xf0) {
		remove_keycode(map[midx][i][j], keycode_arr);
		if (map[midx][i][j] == KEY_CAPS_LOCK)
			remove_keycode(KEY_ESC & 0xff, keycode_arr);
	} else if (type == 0xe0)
		keycode_modifier &= ~(map[midx][i][j] & 0xff);
	else if (type == 0x99) {
		switch(map[midx][i][j] & 0xff) {
		case 5:
		case 6:
		case 7:
			send_mouse_value(0,0,0,0); break;
		}
	} else {
		if (midx == 1) {
			midx = 0;
			keycode_modifier = 0;
			clear_keycode(keycode_arr);
		}
	}
}

int scan_one()
{
	int i, j;
	int ret = 0;
	int flag = 0;

	for (i = 0; i < IOX; i++) {
		int x = io2row[i];

		for (j = 0; j < IOX; j++)
			gpio_set_level(io2row[j], 1);
		gpio_set_level(x, 0);

		vTaskDelay(1 / portTICK_RATE_MS);

		for (j = 0; j < IOY; j++) {
			int y = io2col[j];
			int v = gpio_get_level(y);
			if (v == 0) {
				if (keymap[i][j] == 0 && check(i, j)) {
					keymap[i][j] = 1;
					ret++;
					flag = 1;
					on_keydown(i, j);
				}
			} else {
				if (keymap[i][j] == 1) {
					keymap[i][j] = 0;
					ret--;
					flag = 1;
					on_keyup(i, j);
				}
			}
		}
	}
	if (flag) {
		send_keyboard_value(keycode_modifier, keycode_arr, sizeof(keycode_arr));
	}

	return ret;
}

static void scan_proc(void* arg)
{
	int i;
	uint32_t io_num = 0;

	for (i = 0; i < IOX; i++)
		gpio_set_level(io2row[i], 0);

	for (i = 0; i < IOY; i++) {
		gpio_isr_handler_add(io2col[i], gpio_isr_handler, (void*)io2col[i]);
		gpio_intr_enable(io2col[i]);
	}

	int dcount = 0;
	for (; xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);) {
		int c = 50;
		for (i = 0; i < IOY; i++)
			gpio_intr_disable(io2col[i]);

		ESP_LOGI(TAG, "from %d", io_num);

		do {
			dcount += scan_one();
			vTaskDelay(25 / portTICK_RATE_MS);
		} while (dcount || (--c >= 0));

		for (i = 0; i < IOX; i++)
			gpio_set_level(io2row[i], 0);

		for (i = 0; i < IOY; i++)
			gpio_intr_enable(io2col[i]);

		ESP_LOGI(TAG, "sleep");
	}
}

void app_main_gpio()
{
	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT | GPIO_MODE_DEF_OD;
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL | (1<<12) | (1<<27) | (1<<0) | (1<<2) | (1<<15);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));

	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

	xTaskCreate(scan_proc, "scan_proc", 2048, NULL, 1, NULL);
}
