#include "mesh_device_app.h"

#include "esp_log.h"

#include "sdkconfig.h"

#define TAG "MESH-DEVICE-APP"

esp_err_t ble_mesh_device_init(void) {
    #if CONFIG_COENV_NODE_TYPE_SENSOR
    // INIT CLIENT MODEL
    return ble_mesh_device_init_client();

    #elif CONFIG_COENV_NODE_TYPE_GATEWAY
    //Init Sensor MOdel
    return ble_mesh_device_init_server();
    #else
    // Invalid configuration
    ESP_LOGE(TAG, "Invalid Kconfig Device Type! Please reconfigure your project");
    return ESP_FAIL;
    #endif
}