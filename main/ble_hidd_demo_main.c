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
#include "hid_dev.h"

#include "common.h"

#define GATTS_TAG "FABI/FLIPMOUSE"

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

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

void send_keyboard_value(key_mask_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key)
{
    esp_hidd_send_keyboard_value(hid_conn_id, special_key_mask, keyboard_cmd, num_key);
}

void send_mouse_value(uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y, int8_t wheel)
{
    esp_hidd_send_mouse_value(hid_conn_id, mouse_button, mickeys_x, mickeys_y, wheel);
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
