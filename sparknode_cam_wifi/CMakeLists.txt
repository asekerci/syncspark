# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Set the path to your extra components
set(EXTRA_COMPONENT_DIRS ../components/sys_utils
                         ../components/esp32_chip_info 
                         ../components/esp32cam_leds
                         ../components/wifi_net 
                         ../components/ota_utils
                         ../components/nvs_utils
                         ../components/esp32cam_ov2640
   )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sparknode_cam_wifi)
