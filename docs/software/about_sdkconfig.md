# `sdkconfig` and Build System Integration

When building a project, the ESP-IDF build system

1.  Loads default configuration values from Kconfig files,
2.  Applies any overrides from `sdkconfig.defaults`, and finally
3.  Loads settings from `sdkconfig` if it exists.

Values in `sdkconfig.defaults` serve as additional defaults and are ignored if
the corresponding option is already defined in `sdkconfig`. This ensures that
manual changes made in `sdkconfig` (for example, via menuconfig) always take
precedence.

The `sdkconfig` file can be generated or updated in two ways:

-   **Interactively:** Using the `idf.py menuconfig` command, which launches a
    text-based interface for configuring project settings related to ESP-IDF
    components and the target chip. After saving changes in this interface, the
    `sdkconfig` file in the project's root directory is updated to reflect the
    new configuration.
-   **Non-interactively:** By running `idf.py reconfigure`, which (re)generates
    the `sdkconfig` file based on the current configuration and Kconfig
    defaults, without launching the interactive menu.

Also note that, if you change the target chip using
`idf.py set-target <target>`, the build directory is cleared and a new
`sdkconfig` file is created for the selected target. The previous configuration
is saved as `sdkconfig.old`.
