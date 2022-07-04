# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
# Include the subsidiary .cmake file to get the SDK
include(pico_sdk_import.cmake)
 
# Set the name and version of the project
project(playground VERSION 1.0.0)
 
# Link the Project to a source file (step 4.6)
add_executable(playground source.c)
 
# Link the Project to an extra library (pico_stdlib)
target_link_libraries(playground 
    pico_stdlib
    hardware_dma
    hardware_pio
)
 
# Initalise the SDK
pico_sdk_init()

# Build pio
pico_generate_pio_header(playground ${CMAKE_CURRENT_LIST_DIR}/n64_pi.pio)
pico_generate_pio_header(playground ${CMAKE_CURRENT_LIST_DIR}/n64_data_tester.pio)
pico_generate_pio_header(playground ${CMAKE_CURRENT_LIST_DIR}/n64_data_control.pio)
 
# Enable USB, and DISABLE UART output
pico_enable_stdio_usb(playground 1)
pico_enable_stdio_uart(playground 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(playground)

# Enable extra outputs (SWD?)
pico_add_extra_outputs(playground)
