/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill
 */

#pragma once

#define QSPI_SCLK_PIN 0 // sio_30, clock
#define QSPI_SS_PIN   1 // sio_31, chip select
#define QSPI_SD0_PIN  2 // sio_32, data 0
#define QSPI_SD1_PIN  3 // sio_33, data 1
#define QSPI_SD2_PIN  4 // sio_34, data 2
#define QSPI_SD3_PIN  5 // sio_35, data 3

#define CMD_FAST_READ_QUAD 0xEB
#define CMD_FAST_WRITE_QUAD 0x38

typedef struct RamArrayConfig {
    // Chip specs and array configuration
    uint32_t byteCapacity;    // 8MB = 8388608 bytes
    uint32_t numChipsInArray; // 8 Chips 
    uint32_t clockSpeed;      // 100MHZ
    uint32_t waitCycles;      // Num of cycles to wait for a given mode, 6 for qspi

    // Chip commands
    unsigned char CMD_READ;   // Fast read quad = 0xEB
    unsigned char CMD_WRITE;   // Quad write = 0x38

    // Pin configuration
    uint8_t clk_pin;          // Clock pin
    uint8_t cs_mux_pin0;      // Chip select mux pin 0
    uint8_t cs_mux_pin1;      // Chip select mux pin 1
    uint8_t cs_mux_pin2;      // Chip select mux pin 2
    uint8_t ie_pin;           // Input enable pin. 0 = All chip selects brought high. 1 = only the specified CS will be driven low 
    
    // Pins are on qspi pin numbers
    uint8_t io_pin_0;         // I/O Pin 0
    uint8_t io_pin_1;         // I/O Pin 1
    uint8_t io_pin_2;         // I/O Pin 2
    uint8_t io_pin_3;         // I/O Pin 3
} RamArrayConfig;

/*
 * Standard 8 chip configuration found on the v2 picocard64 boards
 * IMPORTANT! Copy this configuration and set the pins according to the MCU reference
 */
/* Use this default configuration
RamArrayConfig v2StandardDefaultConfig = {
    .byteCapacity = 8388608,
    .numChipsInArray = 8,
    .clockSpeed = 84 * 1000000, // 84 MHZ. Just use the lower bound as default because we may cross a 1k page boundry and the chip internally limits the speed to 84MHZ in this case.
    .waitCycles = 6,
    .CMD_READ = CMD_FAST_READ_QUAD,
    .CMD_WRITE = CMD_FAST_WRITE_QUAD,
    // Pin configuration will vary based on which MCU is referencing
    .clk_pin = QSPI_SCLK_PIN,
    .cs_mux_pin0 = 0,
    .cs_mux_pin1 = 0,
    .cs_mux_pin2 = 0,
    .ie_pin = 0,
    .io_pin_0 = QSPI_SD0_PIN,
    .io_pin_1 = QSPI_SD1_PIN,
    .io_pin_2 = QSPI_SD2_PIN,
    .io_pin_3 = QSPI_SD3_PIN,
};
*/

void init_ram_array(RamArrayConfig ramArrayConfig);

/// @brief Read data into a buffer from the psram array.
/// @param ramArray 
/// @param address 
/// @param bytesToRead 
/// @param buffer 
void psramArray_read(RamArrayConfig ramArray, uint32_t address, uint32_t bytesToRead, uint32_t* buffer);

/// @brief Write data to a starting address from a buffer into the psram array
/// @param ramArray 
/// @param address 
/// @param bytesToWrite 
/// @param buffer 
void psramArray_write(RamArrayConfig ramArray, uint32_t address, uint32_t bytesToWrite, uint32_t* buffer);
