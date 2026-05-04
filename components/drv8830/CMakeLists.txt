# CMakeLists.txt for Custom ESP-IDF Component

# idf_component_register is the primary function for registering a component
# It tells the ESP-IDF build system about your component's source files and 
# include directories
idf_component_register(
    # SRCS: Source files to compile (relative to the component directory)
    SRCS 
        "src/drv8830.c"
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
    	esp_driver_i2c
    #     esp_http_client  # Example of requiring another ESP-IDF component

    PRIV_REQUIRES
        driver
        syncspark_config
)

# Optional: Additional compiler definitions or flags specific to this component
# target_compile_definitions(${COMPONENT_LIB} PRIVATE
#     MY_CUSTOM_DEFINITION=1
# )

# Optional: Additional compiler options for this component
# target_compile_options(${COMPONENT_LIB} PRIVATE
#     -Wall -Werror
# )

# About the PRIVE_REQUIRES section:
# The PRIV_REQUIRES driver line in the idf_component_register function specifies that this component 
# has a private dependency on the driver component in the ESP-IDF framework.

# Key Points:

# Private Dependency:
# The PRIV_REQUIRES keyword indicates that the dependency is private. This means that 
# the driver component is required to build this component, but it will not expose the driver 
# component's include paths or libraries to other components that depend on this one.

# driver Component:
# The driver component is a standard ESP-IDF component that provides low-level drivers for 
# peripherals (e.g., GPIO, I2C, SPI, etc.). By declaring this dependency, the build system 
# ensures that the driver component is included and available when building this component.

# Build System Behavior:
# The ESP-IDF build system will:
# - Ensure the driver component is built before this component.
# - Make the driver component's headers and libraries available only to this component during 
#   compilation.
