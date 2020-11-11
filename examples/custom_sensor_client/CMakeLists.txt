# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/button
#                          $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/example_init)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/bluetooth/esp_ble_mesh/common_components/button)

list(APPEND EXTRA_COMPONENT_DIRS "../..")
list(APPEND EXTRA_COMPONENT_DIRS components/esp32_SGP30)



project(client)
