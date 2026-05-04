# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ../components/esp32_chip_info 
                         ../components/syncspark_config
                         ../components/sys_utils
                         ../components/ota_utils
                         ../components/nvs_utils
                         ../components/esp32cam_leds
                         ../components/wifi_net
                         ../components/rgb_led_ring
                         ../components/matrix_utils
                         ../components/icm20948
   )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sparknode_ota_updater)
