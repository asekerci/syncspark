# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Set the path to your extra components
set(EXTRA_COMPONENT_DIRS ../components/esp32_chip_info 
						 ../components/esp32cam_leds 
                         ../components/nvs_utils
    )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(get_esp32_chip_info)

