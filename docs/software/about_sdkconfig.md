# `sdkconfig` and Build System Integration

ESP-IDF stores the project's configuration in the `sdkconfig` file located in
the project root.

Configuration values originate from three places:

1. Default values defined in the ESP-IDF Kconfig files.
2. Optional overrides in `sdkconfig.defaults`.
3. User-selected values stored in `sdkconfig`.

If `sdkconfig` already exists, it is treated as the authoritative project
configuration. Any configuration options that are missing (for example, because
a newer ESP-IDF version introduced additional options) are initialized from
`sdkconfig.defaults` if present, otherwise from the Kconfig defaults. The
updated configuration is then written back to `sdkconfig`.

The `sdkconfig` file can be created or updated in several ways:

- **Interactively:** `idf.py menuconfig` launches the configuration interface.
  When the configuration is saved, the `sdkconfig` file is updated.

- **Non-interactively:** `idf.py reconfigure` reruns the configuration process
  and updates `sdkconfig` if necessary, without opening the interactive menu.

When changing the target chip using

    idf.py set-target <target>

the build directory is deleted, the existing `sdkconfig` is preserved as
`sdkconfig.old`, and a new `sdkconfig` is generated for the selected target
using the available defaults and any applicable existing settings.

## Recommended Workflow

1. `idf.py menuconfig` (this is the interactive step)
2. Build and test the board, go back to (1) until all board parameters
   are set properly.
3. `idf.py save-defconfig` (this command will generate a `sdkconfig.defaults`)
4. Inspect `sdkconfig.defaults`
5. Commit `sdkconfig.defaults to Git`

The inspection step is important because save-defconfig sometimes includes
options we don't really care about (for example, settings that happened
to change between ESP-IDF releases). We can safely remove those if we
don't want to lock them in.
