cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt ble-mesh-custom-sensor-model)