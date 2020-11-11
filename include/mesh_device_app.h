
#ifndef __MESH_DEVICE_APP_H__
#define __MESH_DEVICE_APP_H__


#include "mesh_client.h"
#include "mesh_server.h"


/**
 * @brief Initializes Custom Sensor Model based on Kconfig choice (Client or Server)
 * 
 * @note  The device type must be chosen on menuconfig component variable 
 */ 
esp_err_t ble_mesh_device_init(void);


#endif  // __MESH_DEVICE_APP_H__
