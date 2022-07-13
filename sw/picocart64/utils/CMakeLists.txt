# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
 
# Set the name and version of the project
project(picocart_utils VERSION 1.0.0)
 
# Link the Project to a source file (step 4.6)
add_executable(picocart_utils picocart_utils.c)
 
# Link the Project to an extra library (pico_stdlib)
target_link_libraries(picocart_utils 
    pico_stdlib
    hardware_dma
    hardware_pio
    hardware_spi
)
 
# Initalise the SDK
pico_sdk_init()

# Build pio
pico_generate_pio_header(picocart_utils ${CMAKE_CURRENT_LIST_DIR}/n64_data_tester.pio)
#pico_generate_pio_header(picocart_utils ${CMAKE_CURRENT_LIST_DIR}/n64_data_control.pio)
 
# Enable USB, and DISABLE UART output
pico_enable_stdio_usb(picocart_utils 1)
pico_enable_stdio_uart(picocart_utils 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(picocart_utils)

# Enable extra outputs (SWD?)
pico_add_extra_outputs(picocart_utils)
