/**
 * @file mesh_client.h
 * 
 * @brief
 * 
 * @author
 * 
 * @date  11/2020
 */

#ifndef __MESH_CLIENT_H__
#define __MESH_CLIENT_H__

#include <stdio.h>
#include <string.h>

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

// #include "ble_mesh_example_init.h"



#define BLE_MESH_DEVICE_NAME    "COEnv Node"    /*!< Device Advertising Name */ 
#define CID_ESP                 0x02E5          /*!< Espressif Component ID */

//* Definicao dos IDs dos Models (Server e Client)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_ID_SERVER      0x1414  //! (tem que ver dps se esse valor nao é de algum que ja existe)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_ID_CLIENT      0x1415  

//* Definimos os OPCODES das mensagens (igual no server)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_OP_GET         ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_OP_SET         ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define ESP_BLE_MESH_CUSTOM_SENSOR_MODEL_OP_STATUS      ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)

#define ESP_BLE_MESH_GROUP_PUB_ADDR                     0xC100


/** 
 * @brief Device Main Data Structure
 */
typedef struct {
    char device_name[6];
    
    /**< BME280 Data */
    uint16_t temperature;   /*!< BME260 calibrated temperature */
    uint16_t pressure;      /*!< BME260 calibrated pressure */
    uint16_t humidity;      /*!< BME260 calibrated humidity */

    /**< SGP30 Data */
    uint16_t tVOC;          /*!< SGP30 total volatile organic compound */
    uint16_t eCO2;          /*!< SGP30 equivalent CO2 */

    /**< Mic Noise */
    uint16_t noise_level;   /*!< Analog microphone noise level */

    //! Add AS7262 data (float?)
} __attribute__((packed)) model_sensor_data_t;


/**
 * @brief Initializes BLE Mesh stack, initializing Models and it's callback functions
 * 
 */
esp_err_t ble_mesh_device_init(void);


/**
 * @brief Custom Sensor Client Model SET message that
 *        publishes data to ESP_BLE_MESH_GROUP_PUB_ADDR
 */ 
esp_err_t ble_mesh_custom_sensor_client_model_message_set(model_sensor_data_t set_data);


/**
 * @brief Custom Sensor Client Model GET message that
 *        publishes data to ESP_BLE_MESH_GROUP_PUB_ADDR
 * 
 * @note  Received data will be available on Model Callback function
 */ 
esp_err_t ble_mesh_custom_sensor_client_model_message_get(void);


#endif  // __MESH_CLIENT_H__

