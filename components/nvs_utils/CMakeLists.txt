# components/nvs_utils/CMakeLists.txt

idf_component_register(
    SRCS "src/nvs_utils.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
)
