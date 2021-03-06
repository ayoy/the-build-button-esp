// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
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

/****************************************************************************
*
* This file is for gatt server. It can send adv data, be connected by clent.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. Then two devices will exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "client_id.h"
#include "led_pwm.h"
#include "button.h"

#define BUILDBUTTON_TAG "BUILD_BUTTON"

void update_idle_flag(uint8_t value);


///Declare the static function
static void gatts_trigger_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_idle_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define BBTN_TRIGGER_SERVICE_UUID   0x00FF
#define BBTN_TRIGGER_CHAR_UUID      0xFF01
#define BBTN_TRIGGER_DESCR_UUID     0x3333
#define BBTN_TRIGGER_NUM_HANDLE     4

#define BBTN_IDLE_SERVICE_UUID   0x00EE
#define BBTN_IDLE_CHAR_UUID      0xEE01
#define BBTN_IDLE_DESCR_UUID     0x2222
#define BBTN_IDLE_NUM_HANDLE     4

#define DEVICE_NAME            "Build Button"
#define MANUFACTURER_DATA_LEN  17

#define BBTN_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

uint8_t char_str[] = {0x11,0x22,0x33};
uint8_t char_desc_str[] = "Connected device UUID";//{'S','m','y', 'r'};
esp_gatt_char_prop_t trigger_property = 0;
esp_gatt_char_prop_t idle_property = 0;

esp_attr_value_t trigger_char_val =
{
    .attr_max_len = BBTN_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char_str),
    .attr_value   = char_str,
};

esp_attr_value_t trigger_char_descr_val =
{
    .attr_max_len = BBTN_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char_desc_str),
    .attr_value   = char_desc_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 2
#define TRIGGER_APP_ID 0
#define IDLE_APP_ID 1

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [TRIGGER_APP_ID] = {
        .gatts_cb = gatts_trigger_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [IDLE_APP_ID] = {
        .gatts_cb = gatts_idle_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t trigger_prepare_write_env;
static prepare_type_env_t idle_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

int client_id_set = 0;
uint8_t client_id[128];
size_t client_id_length = 0;
uint8_t is_idle = 1;
const uint8_t kButtonGPIO = 25;
const uint8_t kLEDGPIO = 26;
uint8_t wake_up_handled = 0;

void trigger_action()
{
    ESP_LOGI(BUILDBUTTON_TAG, "notifying :)");

    if (gl_profile_tab[TRIGGER_APP_ID].gatts_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(BUILDBUTTON_TAG, "GATTS interface is not set");
    } else if (client_id_set > 0) {
        ESP_LOGI(BUILDBUTTON_TAG, "sending notification event");
        esp_ble_gatts_send_indicate(gl_profile_tab[TRIGGER_APP_ID].gatts_if,
            gl_profile_tab[TRIGGER_APP_ID].conn_id,
            gl_profile_tab[TRIGGER_APP_ID].char_handle,
            client_id_length, client_id, false);
        update_idle_flag(0);
    }
}


void handle_button_press()
{
    if (is_idle) {
        if (gl_profile_tab[TRIGGER_APP_ID].gatts_if != 0) {
            ESP_LOGE(BUILDBUTTON_TAG, "TRIGGERED FROM BUTTON HANDLER TASK");
            trigger_action();
        }
    } else {
        ESP_LOGE(BUILDBUTTON_TAG, "INTERRUPTING CURRENT RUN ON USER REQUEST");
        update_idle_flag(1);
    }
}

void handle_button_touch_down()
{
    gpio_pad_select_gpio(kLEDGPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(kLEDGPIO, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(kLEDGPIO, GPIO_PULLDOWN_ONLY);

    gpio_set_level(kLEDGPIO, 1);
}

TaskHandle_t deep_sleep_task_handle = NULL;

void enter_deep_sleep()
{
    const uint64_t ext_wakeup_pin_mask = 1ULL << kButtonGPIO;
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask, ESP_EXT1_WAKEUP_ALL_LOW);

    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        esp_bluedroid_disable();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
    }
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    ESP_LOGE(BUILDBUTTON_TAG, "Entering deep sleep");
    esp_deep_sleep_start();
}

void deep_sleep_task(void *pvParameter)
{
    ESP_LOGE(BUILDBUTTON_TAG, "Starting deep sleep task");
    vTaskDelay(20 * 1000 / portTICK_PERIOD_MS);
    enter_deep_sleep();
}

void update_idle_flag(uint8_t value) 
{
    if (value > 1) value = 1;

    if (is_idle != value) {
        is_idle = value;
        if (is_idle) {
            set_led_pwm_enabled(0);
            xTaskCreate(&deep_sleep_task, "deep_sleep_task", 3072, NULL, 5, &deep_sleep_task_handle);
            esp_ble_gap_start_advertising(&adv_params);
        } else {
            set_led_pwm_enabled(1);
            if (deep_sleep_task_handle) {
                ESP_LOGE(BUILDBUTTON_TAG, "Canceling deep sleep task");
                vTaskDelete(deep_sleep_task_handle);
                deep_sleep_task_handle = NULL;
            }
            esp_ble_gap_stop_advertising();
        }
    }
}



static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BUILDBUTTON_TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BUILDBUTTON_TAG, "Advertising stop failed");
        }
        else {
            ESP_LOGI(BUILDBUTTON_TAG, "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(BUILDBUTTON_TAG, "update connetion params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(BUILDBUTTON_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(BUILDBUTTON_TAG, "Send response error");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

static void gatts_trigger_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[TRIGGER_APP_ID].service_id.is_primary = true;
        gl_profile_tab[TRIGGER_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[TRIGGER_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[TRIGGER_APP_ID].service_id.id.uuid.uuid.uuid16 = BBTN_TRIGGER_SERVICE_UUID;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
        if (set_dev_name_ret){
            ESP_LOGE(BUILDBUTTON_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(BUILDBUTTON_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(BUILDBUTTON_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[TRIGGER_APP_ID].service_id, BBTN_TRIGGER_NUM_HANDLE);
        break;
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(BUILDBUTTON_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(BUILDBUTTON_TAG, "GATT_WRITE_EVT, value len %d", param->write.len);
            char buf[128], *pos = buf; 
            for (int i=0; i != param->write.len; i++) {
                if (i) {
                    pos += sprintf(pos, ", ");
                }
                pos += sprintf(pos, "%d", param->write.value[i]);
            }
            ESP_LOGI(BUILDBUTTON_TAG, "GATT_WRITE_EVT, value: %s", buf);
            esp_log_buffer_hex(BUILDBUTTON_TAG, param->write.value, param->write.len);
            if (gl_profile_tab[TRIGGER_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    if (trigger_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(BUILDBUTTON_TAG, "notify enable");

                        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 && wake_up_handled == 0) {
                            wake_up_handled = 1;
                            ESP_LOGE(BUILDBUTTON_TAG, "TRIGGERED FROM WRITE EVENT (NOTIFICATION ENABLE) HANDLER");
                            trigger_action();
                        }
                        // uint8_t notify_data[15];
                        // for (int i = 0; i < sizeof(notify_data); ++i)
                        // {
                        //     notify_data[i] = i%0xff;
                        // }
                        // //the size of notify_data[] need less than MTU size
                        // ESP_LOGI(BUILDBUTTON_TAG, "current_gatts_if: %d, current_conn_id: %d", gatts_if, param->write.conn_id);
                        // esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[TRIGGER_APP_ID].char_handle,
                        //                         sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (trigger_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(BUILDBUTTON_TAG, "indicate enable");
                        // uint8_t indicate_data[15];
                        // for (int i = 0; i < sizeof(indicate_data); ++i)
                        // {
                        //     indicate_data[i] = i%0xff;
                        // }
                        // //the size of indicate_data[] need less than MTU size
                        // esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[TRIGGER_APP_ID].char_handle,
                        //                         sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(BUILDBUTTON_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(BUILDBUTTON_TAG, "unknown descr value");
                    esp_log_buffer_hex(BUILDBUTTON_TAG, param->write.value, param->write.len);
                }
            }
        }
        example_write_event_env(gatts_if, &trigger_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[TRIGGER_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[TRIGGER_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[TRIGGER_APP_ID].char_uuid.uuid.uuid16 = BBTN_TRIGGER_CHAR_UUID;

        esp_ble_gatts_start_service(gl_profile_tab[TRIGGER_APP_ID].service_handle);
        trigger_property = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[TRIGGER_APP_ID].service_handle, &gl_profile_tab[TRIGGER_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ,
                                                        trigger_property,
                                                        &trigger_char_val, NULL);
        if (add_char_ret){
            ESP_LOGE(BUILDBUTTON_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(BUILDBUTTON_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[TRIGGER_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[TRIGGER_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[TRIGGER_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(BUILDBUTTON_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(BUILDBUTTON_TAG, "the gatts demo char length = %x", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(BUILDBUTTON_TAG, "prf_char[%x] =%x",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[TRIGGER_APP_ID].service_handle, &gl_profile_tab[TRIGGER_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &trigger_char_descr_val, NULL);
        if (add_descr_ret){
            ESP_LOGE(BUILDBUTTON_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[TRIGGER_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(BUILDBUTTON_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(BUILDBUTTON_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[TRIGGER_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "ESP_GATTS_DISCONNECT_EVT");
        update_idle_flag(1);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(BUILDBUTTON_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
    case ESP_GATTS_READ_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_DELETE_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
    default:
        break;
    }
}

static void gatts_idle_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[IDLE_APP_ID].service_id.is_primary = true;
        gl_profile_tab[IDLE_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[IDLE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[IDLE_APP_ID].service_id.id.uuid.uuid.uuid16 = BBTN_IDLE_SERVICE_UUID;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[IDLE_APP_ID].service_id, BBTN_IDLE_NUM_HANDLE);
        break;
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(BUILDBUTTON_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(BUILDBUTTON_TAG, "GATT_WRITE_EVT, value len %d, value:", param->write.len);
            esp_log_buffer_hex(BUILDBUTTON_TAG, param->write.value, param->write.len);

            size_t id_length = param->write.len-2;
            if (param->write.len < 3 || param->write.value[id_length] != ':') {
                ESP_LOGE(BUILDBUTTON_TAG, "Idle characteristic payload formatted incorrectly");
            } else {
                int id_length_unchanged = client_id_length == id_length;
                if (client_id_set && id_length_unchanged &&
                    client_id_compare(client_id, param->write.value, client_id_length)) 
                {
                    ESP_LOGE(BUILDBUTTON_TAG, "Known client, setting idle flag");
                } else {
                    ESP_LOGE(BUILDBUTTON_TAG, "Unknown client, storing new Client ID");
                    write_client_id_to_nvs(param->write.value, id_length);
                    fetch_client_id_from_nvs(client_id, &client_id_length);
                    client_id_set = client_id_length > 0;
                }
                uint8_t idle = param->write.value[param->write.len-1];
                update_idle_flag(idle);
            }
        }
        example_write_event_env(gatts_if, &idle_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[IDLE_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[IDLE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[IDLE_APP_ID].char_uuid.uuid.uuid16 = BBTN_IDLE_CHAR_UUID;

        esp_ble_gatts_start_service(gl_profile_tab[IDLE_APP_ID].service_handle);
        idle_property = ESP_GATT_CHAR_PROP_BIT_WRITE;
        esp_err_t add_char_ret =esp_ble_gatts_add_char( gl_profile_tab[IDLE_APP_ID].service_handle, &gl_profile_tab[IDLE_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_WRITE,
                                                        idle_property,
                                                        NULL, NULL);
        if (add_char_ret){
            ESP_LOGE(BUILDBUTTON_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        gl_profile_tab[IDLE_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[IDLE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[IDLE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(gl_profile_tab[IDLE_APP_ID].service_handle, &gl_profile_tab[IDLE_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[IDLE_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(BUILDBUTTON_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[IDLE_APP_ID].conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BUILDBUTTON_TAG, "ESP_GATTS_CONF_EVT status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(BUILDBUTTON_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
    case ESP_GATTS_READ_EVT:
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_DELETE_EVT:
    case ESP_GATTS_DISCONNECT_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(BUILDBUTTON_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    fetch_client_id_from_nvs(client_id, &client_id_length);
    esp_log_buffer_hex(BUILDBUTTON_TAG, client_id, client_id_length);
    client_id_set = client_id_length > 0;

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
        wake_up_handled = client_id_set == 0;

        ESP_LOGE(BUILDBUTTON_TAG, "WAKE UP CAUSE: DEEP SLEEP INTERRUPT");

        set_led_pwm_gpio(kLEDGPIO);
        start_button_task(kButtonGPIO, &handle_button_press, 
            NULL, 3000 / portTICK_PERIOD_MS,
            handle_button_touch_down);

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(BUILDBUTTON_TAG, "%s initialize controller failed", __func__);
            return;
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
        if (ret) {
            ESP_LOGE(BUILDBUTTON_TAG, "%s enable controller failed", __func__);
            return;
        }
        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(BUILDBUTTON_TAG, "%s init bluetooth failed", __func__);
            return;
        }
        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(BUILDBUTTON_TAG, "%s enable bluetooth failed", __func__);
            return;
        }

        ret = esp_ble_gatts_register_callback(gatts_event_handler);
        if (ret){
            ESP_LOGE(BUILDBUTTON_TAG, "gatts register error, error code = %x", ret);
            return;
        }
        ret = esp_ble_gap_register_callback(gap_event_handler);
        if (ret){
            ESP_LOGE(BUILDBUTTON_TAG, "gap register error, error code = %x", ret);
            return;
        }
        ret = esp_ble_gatts_app_register(TRIGGER_APP_ID);
        if (ret){
            ESP_LOGE(BUILDBUTTON_TAG, "gatts app register error, error code = %x", ret);
            return;
        }
        ret = esp_ble_gatts_app_register(IDLE_APP_ID);
        if (ret){
            ESP_LOGE(BUILDBUTTON_TAG, "gatts app register error, error code = %x", ret);
            return;
        }
        esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
        if (local_mtu_ret){
            ESP_LOGE(BUILDBUTTON_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        }

    } else {
        ESP_LOGE(BUILDBUTTON_TAG, "WAKE UP CAUSE: NORMAL BOOT-UP");
        ESP_LOGE(BUILDBUTTON_TAG, "entering deep sleep right after boot-up");
        enter_deep_sleep();
    }
    return;
}
