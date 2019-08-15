/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is 
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR  
   CONDITIONS OF ANY KIND, either express or implied.
*/

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

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"


/* from i2c */
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

static const char *TAG = "wii-nunchuck";

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

/** 
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns 
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor, 
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define WII_NUNCHUCK_TAG "WII_NUNCHUCK"

#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_PIN_SEL  ((1<<GPIO_OUTPUT_IO_0) | (1<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     21
#define GPIO_INPUT_IO_1     27
#define GPIO_INPUT_PIN_SEL  ((1<<GPIO_INPUT_IO_0) | (1<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "ESP32_WII_NUNCHUCK"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 6
#define RW_TEST_LENGTH 2
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM)
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM)
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TX_BUF_ENABLE 1
#define I2C_MASTER_RX_BUF_ENABLE 1

#define WIINUNCHUCK_WHITE_SENSOR_ADDR CONFIG_WIINUNCHUCK_WHITE_ADDR
#define WIINUNCHUCK_WHITE_CMD_START CONFIG_WIINUNCHUCK_WHITE_OPMODE
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

SemaphoreHandle_t print_mux = NULL;

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */

static esp_err_t initialize_wii_nunchuck(i2c_port_t i2c_num)
{
    ESP_LOGI(WII_NUNCHUCK_TAG, "initialize_wii_nunchuck");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x40, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    esp_err_t result = ESP_OK;
    ESP_LOGI(WII_NUNCHUCK_TAG, "i2c_master_read_slave");
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        ESP_LOGI(WII_NUNCHUCK_TAG, "i2c_master_read_slave data reading succeed: %d", ret);
    } else {
        result = ESP_ERR_TIMEOUT;
        ESP_LOGE(WII_NUNCHUCK_TAG, "i2c_master_read_slave data reading failure: %d", ret);
    }
    return result;
}

static esp_err_t i2c_master_write_slave_terminator(i2c_port_t i2c_num)
{
    esp_err_t result = ESP_OK;
    i2c_cmd_handle_t writeCommand = i2c_cmd_link_create();
    i2c_master_start(writeCommand);
    i2c_master_write_byte(writeCommand, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(writeCommand, 0x00, ACK_CHECK_EN);
    i2c_master_stop(writeCommand);
    esp_err_t writeResult = i2c_master_cmd_begin(i2c_num, writeCommand, 1000 / portTICK_RATE_MS);
    if (writeResult == ESP_OK) {
        ESP_LOGI(WII_NUNCHUCK_TAG, "i2c_master_write_slave_terminator send terminator succeed: %d", writeResult);
    } else {
        result = ESP_ERR_TIMEOUT;
        ESP_LOGE(WII_NUNCHUCK_TAG, "i2c_master_write_slave_terminator send terminator failure: %d", writeResult);
    }
    i2c_cmd_link_delete(writeCommand);
    return result;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_ENABLE,
                              I2C_MASTER_TX_BUF_ENABLE, 0);
}

/* i2c test salvage end */
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
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

void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void gpio_task_example(void* arg)
{
    static uint8_t i = 0;
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(WII_NUNCHUCK_TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if(i == 0) {
            ++i;
            }
        }
    }
}

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    ESP_LOGI(WII_NUNCHUCK_TAG, "hidd_event_callback");
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            ESP_LOGI(WII_NUNCHUCK_TAG, "hidd_event_callback.ESP_HIDD_EVENT_REG_FINISH");
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                ESP_LOGI(WII_NUNCHUCK_TAG, "hidd_event_callback.ESP_HIDD_EVENT_REG_FINISH -> OK");
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
                
            } else {
                ESP_LOGI(WII_NUNCHUCK_TAG, "hidd_event_callback.ESP_HIDD_EVENT_REG_FINISH -> NG");
        }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            ESP_LOGI(WII_NUNCHUCK_TAG, "hidd_event_callback.ESP_BAT_EVENT_REG");
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
         break;
        case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(WII_NUNCHUCK_TAG, "hidd_event_callback.ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(WII_NUNCHUCK_TAG, "hidd_event_callback.ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(WII_NUNCHUCK_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(WII_NUNCHUCK_TAG, param->vendor_write.data, param->vendor_write.length);
        }    
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(WII_NUNCHUCK_TAG, "start gap_event_handler");
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SCAN_RESULT_EVT");
        break;
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_AUTH_CMPL_EVT");
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(WII_NUNCHUCK_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(WII_NUNCHUCK_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(WII_NUNCHUCK_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(WII_NUNCHUCK_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    case ESP_GAP_BLE_KEY_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_KEY_EVT");
        break;

     case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SEC_REQ_EVT");
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(WII_NUNCHUCK_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
     break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_PASSKEY_NOTIF_EVT");
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_PASSKEY_REQ_EVT");
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_OOB_REQ_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_IR_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_NC_REQ_EVT");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_UPDATE_DUPLICATE_EXCEPTIONAL_LIST_COMPLETE_EVT:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_UPDATE_DUPLICATE_EXCEPTIONAL_LIST_COMPLETE_EVT");
        break;
    case ESP_GAP_BLE_EVT_MAX:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.ESP_GAP_BLE_EVT_MAX");
        break;
    default:
        ESP_LOGI(WII_NUNCHUCK_TAG, "gap_event_handler.UNKNOWN EVENT: %d", event);
        break;
    }
    ESP_LOGI(WII_NUNCHUCK_TAG, "finish gap_event_handler");
}

void wii_controller_task(void *pvParameters)
{
    ESP_LOGI(WII_NUNCHUCK_TAG, "start wii_controller_task");
    int ret;
    int retWriteTerminate;
    uint32_t task_idx = (uint32_t)pvParameters;
    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t c_button_state = 0x00;
    uint8_t z_button_state = 0x00;

    ESP_LOGW(TAG, "i2c master initialize");
    ret = initialize_wii_nunchuck(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "master initialize succeed");
    } else {
        ESP_LOGE(TAG, "master initialize failed");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(1) {
        xSemaphoreTake(print_mux, portMAX_DELAY);
        ESP_LOGW(TAG, "i2c slave tx buffer full");
        ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        retWriteTerminate = i2c_master_write_slave_terminator(I2C_MASTER_NUM);
	if (retWriteTerminate != ESP_OK) {
            ret = retWriteTerminate;
	}
    
        xSemaphoreGive(print_mux);
        if (sec_conn) {
            ESP_LOGI(WII_NUNCHUCK_TAG, "wii_controller_task.start to send i2c data");
            if (ret == ESP_OK) {
                int8_t mickeys_x = 0; // Joystick - X
		ESP_LOGI(WII_NUNCHUCK_TAG, "send mouse (%d, %d)", data_rd[0], data_rd[1]);
                if (data_rd[0] > 160) {
                    mickeys_x = 50;
                }
                if (data_rd[0] < 100) {
                    mickeys_x = -50;
                }
                int8_t mickeys_y = 0; // Joystick - Y
                if (data_rd[1] > 160) {
                    mickeys_y = -50;
                }
                if (data_rd[1] < 100) {
                    mickeys_y = 50;
                }
                uint8_t mouse_button = 0x00;
                if (c_button_state == 0x02 && (data_rd[5] & 0x02) == 0x00) {
                    mouse_button = mouse_button | 0x01;
		} else if (c_button_state == 0x00 && (data_rd[5] & 0x02) == 0x02) {
                    mouse_button = mouse_button & 0xfe;
		}
                if (z_button_state == 0x01 && (data_rd[5] & 0x01) == 0x00) {
                    mouse_button = mouse_button | 0x02;
		} else if (z_button_state == 0x00 && (data_rd[5] & 0x01) == 0x01) {
                    mouse_button = mouse_button & 0xfd;
		}
                c_button_state = data_rd[5] & 0x02;
                z_button_state = data_rd[5] & 0x01;
        
                ESP_LOGI(WII_NUNCHUCK_TAG, "Send the mouse event");
                // create mouse event
                esp_hidd_send_mouse_value(hid_conn_id, mouse_button, mickeys_x, mickeys_y);
            } else {
                ESP_LOGW(TAG, "TASK[%d] %s: Master read slave error, IO not connected...\n", task_idx, esp_err_to_name(ret));
	    }
        } else {
            ESP_LOGI(WII_NUNCHUCK_TAG, "not connected");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
}

void show_chip_info() {
  uint8_t mac3[6];
  esp_read_mac(mac3, ESP_MAC_WIFI_STA);
  ESP_LOGI(WII_NUNCHUCK_TAG, "[Wi-Fi Station] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]);
                   
  uint8_t mac4[7];
  esp_read_mac(mac4, ESP_MAC_WIFI_SOFTAP);
  ESP_LOGI(WII_NUNCHUCK_TAG, "[Wi-Fi SoftAP] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac4[0], mac4[1], mac4[2], mac4[3], mac4[4], mac4[5]);
                   
  uint8_t mac5[6];
  esp_read_mac(mac5, ESP_MAC_BT);
  ESP_LOGI(WII_NUNCHUCK_TAG, "[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac5[0], mac5[1], mac5[2], mac5[3], mac5[4], mac5[5]);
                   
  uint8_t mac6[6];
  esp_read_mac(mac6, ESP_MAC_ETH);
  ESP_LOGI(WII_NUNCHUCK_TAG, "[Ethernet] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X", mac6[0], mac6[1], mac6[2], mac6[3], mac6[4], mac6[5]);
}

void app_main()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(WII_NUNCHUCK_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(WII_NUNCHUCK_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(WII_NUNCHUCK_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(WII_NUNCHUCK_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(WII_NUNCHUCK_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    show_chip_info();

    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(&wii_controller_task, "wii_controller_task", 2048, NULL, 5, NULL);
}
