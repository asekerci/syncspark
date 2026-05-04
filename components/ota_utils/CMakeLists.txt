# CMakeLists.txt for Custom ESP-IDF Component

# idf_component_register is the primary function for registering a component
# It tells the ESP-IDF build system about your component's source files and 
# include directories
idf_component_register(
    # SRCS: Source files to compile (relative to the component directory)
    SRCS 
        "src/ota_utils.c"
        # You can add multiple source files here if needed
        # "src/another_source_file.c"
    
    # INCLUDE_DIRS: Directories containing header files
    # This allows other components to find and include your headers
    INCLUDE_DIRS 
        "include"
    
    # Optional: Public include directories (if you want to expose
    # headers to other components) PUBLIC_INCLUDE_DIRS
    # "include/public"
    
    # Optional: Dependencies on other components
    REQUIRES 
    	mbedtls app_update esp_http_client esp32cam_leds
    #   esp_http_client  # Example of requiring another ESP-IDF component

    PRIV_REQUIRES
    #   driver esp_event esp_wifi
        esp_timer nvs_utils  # esp_https_ota esp_tls esp_system
)

# Optional: Additional compiler definitions or flags specific to this component
# target_compile_definitions(${COMPONENT_LIB} PRIVATE
#     MY_CUSTOM_DEFINITION=1
# )

# Optional: Additional compiler options for this component
# target_compile_options(${COMPONENT_LIB} PRIVATE
#     -Wall -Werror
# )
