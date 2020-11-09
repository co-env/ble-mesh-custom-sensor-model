cmake_minimum_required(VERSION 3.5)

set(srcs "main.c" 
        "board.c")

# set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/button)

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  "."
                    REQUIRES bt button custom_sensor_client esp32_SGP30)
