cmake_minimum_required(VERSION 3.13)

# Locate the Pico SDK: honour $PICO_SDK_PATH or -DPICO_SDK_PATH=..., otherwise
# use the checkout that sits next to this project.
if (NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif ()
if (NOT PICO_SDK_PATH)
    get_filename_component(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../pico-sdk" ABSOLUTE)
endif ()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Languages must be listed here so the SDK-selected toolchain
# (arm-none-eabi-gcc) is used for C, C++ and the boot2 assembly.
project(hello_world C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Emit build/compile_commands.json so editor IntelliSense (cpptools, clangd)
# gets the SDK include paths, defines and target flags.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pico 2 W = RP2350 + CYW43. Override with -DPICO_BOARD=pico2 etc.
set(PICO_BOARD pico2_w CACHE STRING "Pico board")

pico_sdk_init()

add_executable(pio_uart_dma)
pico_generate_pio_header(pio_uart_dma ${CMAKE_CURRENT_LIST_DIR}/dap.pio)
target_sources(pio_uart_dma PRIVATE dap_probe.c)
target_link_libraries(pio_uart_dma PRIVATE
        pico_stdlib
        hardware_pio
        hardware_dma
        )
pico_add_extra_outputs(pio_uart_dma)

# Console over USB: GPIO0/GPIO1 are used by the PIO UART, and no UART
# adapter is attached.
pico_enable_stdio_usb(pio_uart_dma 1)
pico_enable_stdio_uart(pio_uart_dma 0)

