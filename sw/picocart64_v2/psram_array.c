/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill
 */

#pragma once

/*
chip size = 8MB
Max num chips = 8

0-8MB = 000
8-16  = 001
16-24 = 010
24-32 = 011
32-40 = 100
40-48 = 101
48-56 = 110
56-64 = 111

Total Bytes
67108864 (64MB)

Go to size(in bytes)-1
0 - 8388607 
8388608 - 16777215
16777216 - 25165823
25165824 - 33554431
33554432 - 41943039
41943040 - 50331647
50331648 - 58720255
58720256 - 67108863

PSRAM chip read info (using qpi mode)
Fast Read Quad command ‘hEB 
Command is quad
Addr is quad
wait cycle 6

*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/regs/pads_qspi.h"
#include "psram_array.h"

typedef struct RamArrayOperation {
    uint8_t muxAddress; // e.g. 000, 001, 010, etc 
    uint32_t address; // 24bit address to read from
    uint32_t count; // bytes to read
} RamArrayOperation;

/*
 * Functions to setup, read and write to the qspi pins as gpio
 */

/// @brief Setup the qspi pins as general gpio pins that we can use to communicate to the psram array
void configureQSPIPins() {
    
    // Set input enable on, output disable off
    hw_write_masked(
        &ioqspi_hw->io[QSPI_SS_PIN].ctrl,
        PADS_QSPI_GPIO_QSPI_SS_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SS_IE_BITS | PADS_QSPI_GPIO_QSPI_SS_OD_BITS
    );

    hw_write_masked(
        &ioqspi_hw->io[QSPI_SD0_PIN].ctrl,
        PADS_QSPI_GPIO_QSPI_SD0_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SD0_IE_BITS | PADS_QSPI_GPIO_QSPI_SD0_OD_BITS
    );

    hw_write_masked(
        &ioqspi_hw->io[QSPI_SD1_PIN].ctrl,
        PADS_QSPI_GPIO_QSPI_SD1_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SD1_IE_BITS | PADS_QSPI_GPIO_QSPI_SD1_OD_BITS
    );

    hw_write_masked(
        &ioqspi_hw->io[QSPI_SD2_PIN].ctrl,
        PADS_QSPI_GPIO_QSPI_SD2_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SD2_IE_BITS | PADS_QSPI_GPIO_QSPI_SD2_OD_BITS
    );

    hw_write_masked(
        &ioqspi_hw->io[QSPI_SD3_PIN].ctrl,
        PADS_QSPI_GPIO_QSPI_SD3_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SD3_IE_BITS | PADS_QSPI_GPIO_QSPI_SD3_OD_BITS
    );

    // Set pins as SIO
    ioqspi_hw->io[QSPI_SS_PIN].ctrl = IO_QSPI_GPIO_QSPI_SS_CTRL_FUNCSEL_VALUE_SIO_31 << IO_QSPI_GPIO_QSPI_SS_CTRL_FUNCSEL_LSB;
    ioqspi_hw->io[QSPI_SD0_PIN].ctrl = IO_QSPI_GPIO_QSPI_SD0_CTRL_FUNCSEL_VALUE_SIO_32 << IO_QSPI_GPIO_QSPI_SD0_CTRL_FUNCSEL_LSB;
    ioqspi_hw->io[QSPI_SD1_PIN].ctrl = IO_QSPI_GPIO_QSPI_SD1_CTRL_FUNCSEL_VALUE_SIO_33 << IO_QSPI_GPIO_QSPI_SD1_CTRL_FUNCSEL_LSB;
    ioqspi_hw->io[QSPI_SD2_PIN].ctrl = IO_QSPI_GPIO_QSPI_SD2_CTRL_FUNCSEL_VALUE_SIO_34 << IO_QSPI_GPIO_QSPI_SD2_CTRL_FUNCSEL_LSB;
    ioqspi_hw->io[QSPI_SD3_PIN].ctrl = IO_QSPI_GPIO_QSPI_SD3_CTRL_FUNCSEL_VALUE_SIO_35 << IO_QSPI_GPIO_QSPI_SD3_CTRL_FUNCSEL_LSB;
}

bool gpio1_get(uint gpio) {
    return !!((1ul << gpio) & sio_hw->gpio_hi_in);
}

uint32_t gpio1_get_pins() {
    return sio_hw->gpio_hi_in;
}

void gpio1_set_mask(uint32_t mask) {
    sio_hw->gpio_hi_set = mask;
}

void gpio1_clr_mask(uint32_t mask) {
    sio_hw->gpio_hi_clr = mask;
}

void gpio1_put(uint gpio, bool value) {
    uint32_t mask = 1ul << gpio;
    if (value) {
        gpio1_set_mask(mask);
    } else {
        gpio1_clr_mask(mask);
    }
}

void init_ram_array(RamArrayConfig ramArrayConfig) {
    configureQSPIPins();

    // Do we need to setup an inturrupt to listen to the clk and then deal with sending data?
}

/*
 * PSRAMArray Read/Write functions
 *
 * All Reads & Writes must be completed by raising CE# high immediately afterwards in order to
 * terminate the active read/write wordline and set the device into standby
 */

// TODO: Make this function generic and not hard coded to only support max 2 read/writes
/// @brief Helper function to calculate number and paramters of a read/write operation that may span multiple ram chips
/// @param ramArray 
/// @param startingAddress 
/// @param bytesToRead 
/// @param operations 
/// @param requiredReads 
void calculateOperations(RamArrayConfig ramArray, uint32_t startingAddress, uint32_t bytesToRead, RamArrayOperation* operations, uint* requiredReads) {
    uint startingChip = startingAddress / (ramArray.byteCapacity-1);
    uint endingChip = (startingAddress + bytesToRead) / (ramArray.byteCapacity-1);
    *requiredReads = (endingChip - startingChip) + 1;

    operations[0].muxAddress = startingChip;
    operations[0].address = startingAddress;
    operations[0].count = *requiredReads == 1 ? bytesToRead : (ramArray.byteCapacity-1) - (startingAddress);

    if (*requiredReads == 2) {
        operations[1].muxAddress = endingChip;
        operations[1].address = 0;
        operations[1].count = bytesToRead - operations[0].count;
    }
}

void psramArray_read(RamArrayConfig ramArray, uint32_t address, uint32_t bytesToRead, uint32_t* buffer) {
    RamArrayOperation readArray[2];
    uint requiredReads = 0;
    calculateOperations(ramArray, address, bytesToRead, readArray, &requiredReads);

    // Set mux values
    // Send read command
    // Send address
    // Wait cycles
    // Read data

    // CE# Chip select; active low. When CE#=1, chip is in standby state.
    for(uint8_t r = 0; r < requiredReads; r++) {
        // Set chip select for the mux
        gpio_put(ramArray.cs_mux_pin0, (readArray[r].muxAddress & (1 << 2)) >> 2);
        gpio_put(ramArray.cs_mux_pin1, (readArray[r].muxAddress & (1 << 1)) >> 1);
        gpio_put(ramArray.cs_mux_pin2, (readArray[r].muxAddress & (1 << 0)) >> 0);
        
        gpio_put(ramArray.ie_pin, 1); // Enable reads to the chip
        
        // wait for clock high
        while (!gpio1_get(QSPI_SCLK_PIN)) {
            tight_loop_contents();
        }
        // send read command
        gpio1_set_mask((ramArray.CMD_READ >> 4 & 0xF) << 2);
        gpio1_set_mask((ramArray.CMD_READ & 0xF) << 2);

        //Get each nibble from the address and send the bits
        for(int i = 5; i >= 0; i--) {
            uint8_t nibble = (readArray[r].address >> (i * 4)) & 0xF;

            // wait for clock to go high
            while (!gpio1_get(QSPI_SCLK_PIN)) {
                tight_loop_contents();
            }
            gpio1_set_mask(nibble << QSPI_SD0_PIN);
            
            // uint8_t a0 = (nibble & (1 << 3)) >> 3;
            // uint8_t a1 = (nibble & (1 << 2)) >> 2;
            // uint8_t a2 = (nibble & (1 << 1)) >> 1;
            // uint8_t a3 = (nibble & (1 << 0)) >> 0;

            // gpio1_put(ramArray.io_pin_0, a0);
            // gpio1_put(ramArray.io_pin_1, a1);
            // gpio1_put(ramArray.io_pin_2, a2);
            // gpio1_put(ramArray.io_pin_3, a3);
        }

        // wait 6 cycles
        uint cycleCount = 0;
        bool pinLow = !gpio1_get(QSPI_SCLK_PIN);
        while (cycleCount < 6) {
            bool clk = gpio1_get(QSPI_SCLK_PIN);
            if (pinLow && clk) {
                cycleCount++;
                pinLow = false;
            } else {
                pinLow = true;
            }
        }

        // wait for clk low (read on falling edge)
        while (gpio1_get(QSPI_SCLK_PIN)) {
            tight_loop_contents();
        }
 
        uint32_t readChunk = 0; 
        uint8_t nibbleChunksRead = 0;
        const uint8_t nibblesNeeded = 4;
        for(int b = readArray[r].count * 2; b > 0; b = b - 4) {
            uint8_t nibble = gpio1_get_pins() >> 2; // we don't want the clk or cs pin data: pins 0,1, so drop them
            if (nibbleChunksRead != nibblesNeeded) {
                readChunk |= nibble;
                ++nibbleChunksRead;
            }

            if (nibbleChunksRead == nibblesNeeded) {
                // write the byte to the buffer
                uint32_t bufferIndex = b / nibblesNeeded; // index should be b / 4 where b is nibbles to read and 4 = nibblesNeeded (per 32bit value)
                buffer[bufferIndex] = readChunk; 
                nibbleChunksRead = 0;
                readChunk = 0;
            }
        }

        gpio_put(ramArray.ie_pin, 0); // Finished with this read
    }
}

void psramArray_write(RamArrayConfig ramArray, uint32_t address, uint32_t bytesToWrite, uint32_t* buffer) {
    RamArrayOperation writeArray[2];
    uint requiredWrites = 0;
    calculateOperations(ramArray, address, bytesToWrite, writeArray, &requiredWrites);

    for(uint8_t w = 0; w < requiredWrites; w++) {
        // Set chip select for the mux
        gpio_put(ramArray.cs_mux_pin0, (writeArray[w].muxAddress & (1 << 2)) >> 2);
        gpio_put(ramArray.cs_mux_pin1, (writeArray[w].muxAddress & (1 << 1)) >> 1);
        gpio_put(ramArray.cs_mux_pin2, (writeArray[w].muxAddress & (1 << 0)) >> 0);
        
        gpio_put(ramArray.ie_pin, 1); // Enable writes to the chip
        
        // wait for clock high
        while (!gpio1_get(QSPI_SCLK_PIN)) {
            tight_loop_contents();
        }
        // send read command
        gpio1_set_mask((ramArray.CMD_WRITE >> 4 & 0xF) << 2);
        gpio1_set_mask((ramArray.CMD_WRITE & 0xF) << 2);

        //Get each nibble from the address and send the bits
        for(int i = 5; i >= 0; i--) {
            uint8_t nibble = (writeArray[w].address >> (i * 4)) & 0xF;

            // wait for clock to go high
            while (!gpio1_get(QSPI_SCLK_PIN)) {
                tight_loop_contents();
            }
            gpio1_set_mask(nibble << QSPI_SD0_PIN);
        }

        // wait 6 cycles
        uint cycleCount = 0;
        bool pinLow = !gpio1_get(QSPI_SCLK_PIN);
        while (cycleCount < 6) {
            bool clk = gpio1_get(QSPI_SCLK_PIN);
            if (pinLow && clk) {
                cycleCount++;
                pinLow = false;
            } else {
                pinLow = true;
            }
        }

        // wait for clk low (read on falling edge)
        while (gpio1_get(QSPI_SCLK_PIN)) {
            tight_loop_contents();
        }

        //Get each nibble from the address and send the bits
        for(int i = bytesToWrite; i >= 0; i--) {
            uint32_t nibble1 = buffer[i] >> 4 & 0xF;
            uint32_t nibble2 = buffer[i] & 0xF;

            // wait for clock to go high
            while (!gpio1_get(QSPI_SCLK_PIN)) {
                tight_loop_contents();
            }
            // send first nibble
            gpio1_set_mask(nibble1 << QSPI_SD0_PIN);

            // send second nibble
            gpio1_set_mask(nibble2 << QSPI_SD0_PIN);
        }

        gpio_put(ramArray.ie_pin, 0); // Finished with this write
    }
}