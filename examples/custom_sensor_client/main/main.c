/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <sdkconfig.h>
#include "nvs_flash.h"

#include "esp_log.h"

// #include "mesh_client.h"
#include "mesh_device_app.h"
#include "board.h"

#include "SGP30.h"

#define TAG "MAIN"

sgp30_t main_sensor;

static uint8_t ticks = 0;


static void air_sensor_task(void *arg) {
    ESP_LOGI(TAG, "SGP30 main task initializing...");

    // if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        sgp30_init(&main_sensor);
        // xSemaphoreGive(xSemaphore);
    // }

    // SGP30 needs to be read every 1s and sends TVOC = 400 14 times when initializing
    for (int i = 0; i < 14; i++) {
        vTaskDelay(1000 / portTICK_RATE_MS);

        // if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            sgp30_IAQ_measure(&main_sensor);
            // xSemaphoreGive(xSemaphore);
        // }

        ESP_LOGI(TAG, "SGP30 Calibrating... TVOC: %d,  eCO2: %d",  main_sensor.TVOC, main_sensor.eCO2);
    }

    // Read initial baselines 
    uint16_t eco2_baseline, tvoc_baseline;


    // if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        sgp30_get_IAQ_baseline(&main_sensor, &eco2_baseline, &tvoc_baseline);
        // xSemaphoreGive(xSemaphore);
    // }
    
    ESP_LOGI(TAG, "BASELINES - TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);


    ESP_LOGI(TAG, "SGP30 main task is running...");
    while(1) {
        vTaskDelay(1000 / portTICK_RATE_MS);

        // if (xSemaphoreTake(xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            sgp30_IAQ_measure(&main_sensor);
            // xSemaphoreGive(xSemaphore);
        // }

        ESP_LOGI(TAG, "TVOC: %d,  eCO2: %d",  main_sensor.TVOC, main_sensor.eCO2);
        
        if (ticks++ >= 10) {
            device_sensor_data.eCO2 = main_sensor.eCO2;
            device_sensor_data.tVOC = main_sensor.TVOC;

            ble_mesh_custom_sensor_client_model_message_set(device_sensor_data);

            ticks = 0;
        }
    }
}



void app_main(void) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    // esp_log_level_set("*", ESP_LOG_VERBOSE);

    board_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // err = bluetooth_init();
    // if (err) {
    //     ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
    //     return;
    // }

    // ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    // err = ble_mesh_device_init_client();
    err = ble_mesh_device_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    xTaskCreate(air_sensor_task, "air_sensor_main_task", 1024 * 2, (void *)0, 15, NULL);

}
