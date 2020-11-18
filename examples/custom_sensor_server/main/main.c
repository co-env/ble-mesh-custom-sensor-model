#include <sdkconfig.h>
#include "nvs_flash.h"

#include "esp_log.h"

// #include "mesh_server.h"
#include "mesh_device_app.h"

static const char* TAG = "MESH-SERVER-EXAMPLE";

QueueHandle_t ble_mesh_received_data_queue = NULL;


static void read_received_items(void *arg) {
    ESP_LOGI(TAG, "Task initializing..");

    model_sensor_data_t _received_data;

    while (1) {
        vTaskDelay(500 / portTICK_RATE_MS);
        
        if (xQueueReceive(ble_mesh_received_data_queue, &_received_data, 1000 / portTICK_RATE_MS) == pdPASS) {
            ESP_LOGI(TAG, "Recebido dados de %s", _received_data.device_name);
            ESP_LOGI(TAG, "    Temperatura: %f", _received_data.temperature);
            ESP_LOGI(TAG, "    Pressao:     %f", _received_data.pressure);
            ESP_LOGI(TAG, "    Umidade:     %f", _received_data.humidity);
            ESP_LOGI(TAG, "    TVOC:        %d", _received_data.tVOC);
            ESP_LOGI(TAG, "    eCO2:        %d", _received_data.eCO2);
            ESP_LOGI(TAG, "    Ruido:       %d", _received_data.noise_level);
            ESP_LOGI(TAG, "    Vermelho:    %f", _received_data.red);
            ESP_LOGI(TAG, "    Laranja:     %f", _received_data.orange);
            ESP_LOGI(TAG, "    Amarelo:     %f", _received_data.yellow);
            ESP_LOGI(TAG, "    Verde:       %f", _received_data.green);
            ESP_LOGI(TAG, "    Azul:        %f", _received_data.blue);
            ESP_LOGI(TAG, "    Violeta:     %f", _received_data.violet);
        } else {
            ESP_LOGE(TAG, "Fila vazia? - %s", _received_data.device_name);
        }

        
    }   
} 

void app_main(void) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    ble_mesh_received_data_queue = xQueueCreate(5, sizeof(model_sensor_data_t));

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // err = ble_mesh_device_init_server();
    err = ble_mesh_device_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err 0x%06x)", err);
    }

    xTaskCreate(read_received_items, "Read queue task", 1024 * 2, (void *)0, 20, NULL);

}