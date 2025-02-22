cmake_minimum_required(VERSION 3.12)

# Set the external flash to 16 MB
add_definitions(-DPICO_FLASH_SIZE_BYTES=16777216)

# Build as release but with debug symbols
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(dreamdrive64 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Copy program to ram during boot
set(PICO_COPY_TO_RAM 1)

# add_compile_options(
#         -Wall
#         -save-temps
#         -Wno-unused-function # we have some for the docs that aren't called
#         -Wno-maybe-uninitialized
#         -O2
# )

# Initialize the SDK
pico_sdk_init()

# Include FreeRTOS integration
add_subdirectory(lib/freertos-kernel/portable/ThirdParty/GCC/RP2040 FreeRTOS-Kernel)

# Include 3rd party SDCard implementation
add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/src)

# Add shared code
add_subdirectory(n64_cic)
add_subdirectory(n64_pi)
add_subdirectory(stdio_async_uart)
add_subdirectory(dreamdrive64_shared)

# Add applications
# add_subdirectory(picocart64_v1)
add_subdirectory(dreamdrive64)
#add_subdirectory(dualpico)
