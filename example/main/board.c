/* board.c - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

// #include "driver/uart.h"
#include "esp_log.h"
#include <sdkconfig.h>

#include "board.h"


#include "iot_button.h"

#define TAG "BOARD"

#define SET_BUTTON_IO_NUM           GPIO_NUM_25
#define GET_BUTTON_IO_NUM           GPIO_NUM_26
#define BUTTON_ACTIVE_LEVEL     0

// extern esp_err_t ble_mesh_custom_sensor_client_model_message_set(model_sensor_data_t set_data);
// extern esp_err_t ble_mesh_custom_sensor_client_model_message_get(void);

model_sensor_data_t device_sensor_data;

static void button1_tap_callback(void* arg) {
    ESP_LOGI(TAG, "Button 1 Pressed (%s)", (char *)arg);

    ble_mesh_custom_sensor_client_model_message_get();
}

static void button2_tap_callback(void* arg) {
    ESP_LOGI(TAG, "Button 2 Pressed (%s)", (char *)arg);

    esp_err_t err;

    err = ble_mesh_custom_sensor_client_model_message_set(device_sensor_data);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro na funcao set");
    }

    device_sensor_data.temperature += 3;
    device_sensor_data.noise_level += 2;
    device_sensor_data.humidity--;
}

static void board_button_init(void) {
    button_handle_t btn_handle = iot_button_create(SET_BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button1_tap_callback, "RELEASE");
    }

    button_handle_t btn_handle2 = iot_button_create(GET_BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle2) {
        iot_button_set_evt_cb(btn_handle2, BUTTON_CB_RELEASE, button2_tap_callback, "RELEASE");
    }
}

void board_init(void) {
    strcpy(device_sensor_data.device_name, "esp_3");
    device_sensor_data.temperature = 30;
    device_sensor_data.pressure = 100;
    device_sensor_data.humidity = 60;
    device_sensor_data.tVOC = 0;
    device_sensor_data.eCO2 = 400;
    device_sensor_data.noise_level = 40;


    board_button_init();
}
