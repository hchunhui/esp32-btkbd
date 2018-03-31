// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Adaptions done:
// Copyright 2017 Benjamin Aigner <beni@asterics-foundation.org>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "config.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "bt_trace.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hid_dev.h"

#include "keylayouts.h"

#define GATTS_TAG "FABI/FLIPMOUSE"

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static uint8_t keycode_modifier;
static uint8_t keycode_arr[6];

static config_data_t config;

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

const char hid_device_name_fabi[] = "FABI";
const char hid_device_name_flipmouse[] = "FLipMouse";
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x30,
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                if(config.bt_device_name_index == 1)
                {
                    esp_ble_gap_set_device_name(hid_device_name_flipmouse);
                } else {
                    esp_ble_gap_set_device_name(hid_device_name_fabi);
                }
                esp_ble_gap_config_adv_data(&hidd_adv_data);
                
            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            hid_conn_id = param->connect.conn_id;
            sec_conn = true; //TODO: right here?!?
            LOG_ERROR("%s(), ESP_HIDD_EVENT_BLE_CONNECT", __func__);
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            hid_conn_id = 0;
            LOG_ERROR("%s(), ESP_HIDD_EVENT_BLE_DISCONNECT", __func__);
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     /*case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             LOG_DEBUG("%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;*/
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        if(param->ble_security.auth_cmpl.success)
        {
            LOG_INFO("status = success, ESP_GAP_BLE_AUTH_CMPL_EVT");
        } else {
            LOG_INFO("status = fail, ESP_GAP_BLE_AUTH_CMPL_EVT");
        }
        break;
    //unused events 
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: break;
    //do we need this? occurs on win10 connect.
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: break;
    
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
        //esp_ble_passkey_reply(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, true, 0x00);
        LOG_INFO("ESP_GAP_BLE_PASSKEY_REQ_EVT");
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT:                                /* OOB request event */
        LOG_INFO("ESP_GAP_BLE_OOB_REQ_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        LOG_INFO("ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        LOG_INFO("ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        LOG_INFO("ESP_GAP_BLE_NC_REQ_EVT");
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should sent the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        LOG_INFO("ESP_GAP_BLE_SEC_REQ_EVT");
        break;
    
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer deivce.
        LOG_INFO("The passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        LOG_INFO("key type = %d", param->ble_security.ble_key.key_type);
        break;
    
    default:
        LOG_WARN("unhandled event: %d",event);
        break;
    }
}

void update_config()
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open("fabi_c", NVS_READWRITE, &my_handle);
    if(err != ESP_OK) ESP_LOGE("MAIN","error opening NVS");
    err = nvs_set_u8(my_handle, "btname_i", config.bt_device_name_index);
    if(err != ESP_OK) ESP_LOGE("MAIN","error saving NVS - bt name");
    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    nvs_close(my_handle);
}

void app_main_bt()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    // Read config
    nvs_handle my_handle;
    ret = nvs_open("fabi_c", NVS_READWRITE, &my_handle);
    if(ret != ESP_OK) ESP_LOGE("MAIN","error opening NVS");
    ret = nvs_get_u8(my_handle, "btname_i", &config.bt_device_name_index);
    if(ret != ESP_OK) 
    {
        ESP_LOGE("MAIN","error reading NVS - bt name, setting to FABI");
        config.bt_device_name_index = 0;
    }
    nvs_close(my_handle);
    

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        LOG_ERROR("%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        LOG_ERROR("%s init bluedroid failed\n", __func__);
        return;
    }
    
    //load HID country code for locale before initialising HID
    hidd_set_countrycode(33);

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        LOG_ERROR("%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    /** Do not use "NONE", HID over GATT requires something more than NONE */
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    /** CAP_OUT & CAP_IO work with Winsh***t, but you need to enter a pin which is shown in "make monitor" */
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_IO;           //set the IO capability to No output No input
    /** CAP_IN: host shows you a pin, you have to enter it (unimplemented now) */
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;           //set the IO capability to No output No input
    
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribut to you,
    and the response key means which key you can distribut to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribut to you, 
    and the init key means which key you can distribut to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

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

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    int v = gpio_get_level(gpio_num);
    if (v == 0)
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
	{KEY_CAPS_LOCK, KEY_Q, KEY_E, KEY_T, KEY_U, KEY_O, KEY_LEFT_BRACE, KEY_PAGE_DOWN, },
	{KEY_LEFT_SHIFT, KEY_W, KEY_R, KEY_Y, KEY_I, KEY_P, KEY_RIGHT_BRACE, KEY_HOME, },
	{KEY_LEFT_CTRL, KEY_A, KEY_D, KEY_G, KEY_J, KEY_L, KEY_QUOTE, KEY_RIGHT_SHIFT, },
	{KEY_LEFT_GUI, KEY_S, KEY_F, KEY_H, KEY_K, KEY_SEMICOLON, KEY_ENTER, KEY_RIGHT_CTRL, },
	{KEY_LEFT_ALT, KEY_Z, KEY_C, KEY_B, KEY_M, KEY_PERIOD, KEY_DELETE, 0, },
	{KEY_SPACE, KEY_X, KEY_V, KEY_N, KEY_COMMA, KEY_SLASH, KEY_BACKSLASH, KEY_RIGHT_ALT, },
	},
};
static int midx;

TickType_t caps_time;

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


static void install_isr()
{
	gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
	gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
	gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
	gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_isr_handler, (void*) GPIO_INPUT_IO_3);
	gpio_isr_handler_add(GPIO_INPUT_IO_4, gpio_isr_handler, (void*) GPIO_INPUT_IO_4);
	gpio_isr_handler_add(GPIO_INPUT_IO_5, gpio_isr_handler, (void*) GPIO_INPUT_IO_5);
	gpio_isr_handler_add(GPIO_INPUT_IO_6, gpio_isr_handler, (void*) GPIO_INPUT_IO_6);
	gpio_isr_handler_add(GPIO_INPUT_IO_7, gpio_isr_handler, (void*) GPIO_INPUT_IO_7);
}

static void uninstall_isr()
{
	gpio_isr_handler_remove(GPIO_INPUT_IO_0);
	gpio_isr_handler_remove(GPIO_INPUT_IO_1);
	gpio_isr_handler_remove(GPIO_INPUT_IO_2);
	gpio_isr_handler_remove(GPIO_INPUT_IO_3);
	gpio_isr_handler_remove(GPIO_INPUT_IO_4);
	gpio_isr_handler_remove(GPIO_INPUT_IO_5);
	gpio_isr_handler_remove(GPIO_INPUT_IO_6);
	gpio_isr_handler_remove(GPIO_INPUT_IO_7);
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
	static uint8_t keycode = 0;
	keycode = code;
	printf("sending %d %x\n", keycode, keycode);
	esp_hidd_send_keyboard_value(hid_conn_id,0,&keycode,1);
	//vTaskDelay(1);
	keycode = 0;
	esp_hidd_send_keyboard_value(hid_conn_id,0,&keycode,1);
}

static void  scan_proc(void* arg)
{
	uint32_t io_num;
	int dcount;
	int i, j;
	int flag;

	for (i = 0; i < IOX; i++) {
		int x = io2row[i];
		gpio_set_level(x, 0);
	}

	install_isr();

	for (; xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY); ) {
		printf("leave interrupt mode %d\n", io_num);
		uninstall_isr();

		dcount = 0;
		do {
			flag = 0;
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
							dcount++;
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
							else {
								if (midx == 0) {
									midx = 1;
									keycode_modifier = 0;
									clear_keycode(keycode_arr);
								}
							}

							printf("key down %d %d (%d)\n", i, j, dcount);
							flag = 1;
						}
					} else {
						if (keymap[i][j] == 1) {
							keymap[i][j] = 0;
							dcount--;
							int type = map[midx][i][j] >> 8;
							if (type == 0xf0) {
								remove_keycode(map[midx][i][j], keycode_arr);
								if (map[midx][i][j] == KEY_CAPS_LOCK)
									remove_keycode(KEY_ESC & 0xff, keycode_arr);
							} else if (type == 0xe0)
								keycode_modifier &= ~(map[midx][i][j] & 0xff);
							else {
								if (midx == 1) {
									midx = 0;
									keycode_modifier = 0;
									clear_keycode(keycode_arr);
								}
							}

							printf("key up %d %d (%d)\n", i, j, dcount);
							flag = 1;
						}
					}
				}
			}
			if (flag) {
				esp_hidd_send_keyboard_value(hid_conn_id, keycode_modifier, keycode_arr, sizeof(keycode_arr));
			}
			vTaskDelay(5 / portTICK_RATE_MS);
		} while (dcount);

		for (i = 0; i < IOX; i++) {
			int x = io2row[i];
			gpio_set_level(x, 0);
		}
		install_isr();
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

	xTaskCreate(scan_proc, "scan_proc", 2048, NULL, 10, NULL);
}

void app_main()
{
	app_main_bt();
	app_main_gpio();
}
