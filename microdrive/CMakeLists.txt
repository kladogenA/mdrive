if (TARGET tinyusb_device)
    add_executable(microdrive
            main.c
            hw_config.c
            )

    # pull in common dependencies
    target_link_libraries(microdrive pico_stdlib FatFs_SPI)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(microdrive 1)
    pico_enable_stdio_uart(microdrive 0)

    # Tell CMake where to find other source code
    # add_subdirectory(../no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(microdrive)

    # add url via pico_set_program_url
    example_auto_set_url(microdrive)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
