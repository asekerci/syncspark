# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ../components/sys_utils
			             ../components/esp32_chip_info 
                         ../components/esp32cam_leds
                         ../components/wifi_net
                         ../components/ota_utils
                         ../components/nvs_utils
                         ../components/matrix_utils
                         ../components/bmp388
                         ../components/icm20948
			             ../components/syncspark_config
                         ../components/madgwick_ahrs
    )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sparknode_icm20948)
