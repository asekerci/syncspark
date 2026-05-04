# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Set the path to your extra components
set(EXTRA_COMPONENT_DIRS ../components/syncspark_config
                         ../components/sys_utils
                         ../components/esp32_chip_info
                         ../components/esp32cam_leds
                         ../components/nvs_utils
						 ../components/matrix_utils
                         ../components/rgb_led_ring
					     ../components/icm20948
   )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(glow_ledring)
