/* board.h - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _BOARD_H_
#define _BOARD_H_

#include "driver/gpio.h"
// #include "esp_ble_mesh_defs.h"

#include "mesh_client.h"

extern model_sensor_data_t device_sensor_data;

void board_init(void);


#endif
