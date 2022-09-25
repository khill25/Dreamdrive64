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

#define QSPI_SCLK_PIN 0 // sio_30, clock
#define QSPI_SS_PIN   1 // sio_31, chip select
#define QSPI_SD0_PIN  2 // sio_32, data 0
#define QSPI_SD1_PIN  3 // sio_33, data 1
#define QSPI_SD2_PIN  4 // sio_34, data 2
#define QSPI_SD3_PIN  5 // sio_35, data 3

#define CMD_FAST_READ_QUAD 0xEB

typedef struct RamArrayConfig {
    // Chip specs and array configuration
    uint32_t byteCapacity;    // 8MB = 8388608 bytes
    uint32_t numChipsInArray; // 8 Chips 
    uint32_t clockSpeed;      // 100MHZ

    // Chip commands
    unsigned char CMD_READ;   // Fast read quad = 0xEB
    unsigned char CMD_WRITE;   // Quad write = 0x38

    // Pin connfiguration
    uint8_t clk_pin;          // Clock pin
    uint8_t cs_mux_pin0;      // Chip select mux pin 0
    uint8_t cs_mux_pin1;      // Chip select mux pin 1
    uint8_t cs_mux_pin2;      // Chip select mux pin 2
    
    // Pins are on qspi pin numbers
    uint8_t io_pin_0;         // I/O Pin 0
    uint8_t io_pin_1;         // I/O Pin 1
    uint8_t io_pin_2;         // I/O Pin 2
    uint8_t io_pin_3;         // I/O Pin 3
} RamArrayConfig;

typedef struct RamArrayRead {
    uint8_t muxAddress; // e.g. 000, 001, 010, etc 
    uint32_t address; // 24bit address to read from
    uint32_t bytesToRead; // bytes to read
} RamArrayRead;

/*
 * Standard 8 chip configuration found on the v2 picocard64 boards
 * IMPORTANT! Copy this configuration and set the pins according to the MCU reference
 */
RamArrayConfig v2StandardDefaultConfig = {
    .byteCapacity = 8388608,
    .numChipsInArray = 8,
    .clockSpeed = 100 * 1000000, // 100 MHZ
    .CMD_READ = 0xEB,
    .CMD_WRITE = 0x38,
    // Pin configuration will vary based on which MCU is referencing
    .clk_pin = QSPI_SCLK_PIN,
    .cs_mux_pin0 = 0,
    .cs_mux_pin1 = 0,
    .cs_mux_pin2 = 0,
    .io_pin_0 = QSPI_SD0_PIN,
    .io_pin_1 = QSPI_SD1_PIN,
    .io_pin_2 = QSPI_SD2_PIN,
    .io_pin_3 = QSPI_SD3_PIN,
};

/*
 * Functions to setup, read and write to the qspi pins as gpio
 */
/// @brief Setup the qspi pins as general gpio pins that we can use to communicate to the psram array
void configureQSPIPins() {
    
    // Set input enable on, output disable off
    hw_write_masked(
        &ioqspi_hw->io[QSPI_SS_PIN],
        PADS_QSPI_GPIO_QSPI_SS_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SS_IE_BITS | PADS_QSPI_GPIO_QSPI_SS_OD_BITS
    );

    hw_write_masked(
        &ioqspi_hw->io[QSPI_SD0_PIN],
        PADS_QSPI_GPIO_QSPI_SD0_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SD0_IE_BITS | PADS_QSPI_GPIO_QSPI_SD0_OD_BITS
    );

    hw_write_masked(
        &ioqspi_hw->io[QSPI_SD1_PIN],
        PADS_QSPI_GPIO_QSPI_SD1_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SD1_IE_BITS | PADS_QSPI_GPIO_QSPI_SD1_OD_BITS
    );

    hw_write_masked(
        &ioqspi_hw->io[QSPI_SD2_PIN],
        PADS_QSPI_GPIO_QSPI_SD2_IE_BITS,
        PADS_QSPI_GPIO_QSPI_SD2_IE_BITS | PADS_QSPI_GPIO_QSPI_SD2_OD_BITS
    );

    hw_write_masked(
        &ioqspi_hw->io[QSPI_SD3_PIN],
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

/*
 * PSRAMArray Read/Write functions
 */

void psramArray_read(RamArrayConfig ramArray, uint32_t address, uint32_t bytesToRead, uint32_t* buffer) {
    RamArrayRead readArray[2];
    uint requiredReads = 0;
    calculateReads(ramArray, address, bytesToRead, readArray, &requiredReads);

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
        
        // wait for clock high
        while (!gpio1_get(QSPI_SCLK_PIN)) {
            tight_loop_contents();
        }
        // send read command
        gpio1_set_mask((CMD_FAST_READ_QUAD >> 4 & 0xF) << 2);
        gpio1_set_mask((CMD_FAST_READ_QUAD & 0xF) << 2);

        //Get each nibble from the address and send the bits
        for(int i = 5; i >= 0; i--) {
            uint8_t nibble = (readArray[r].address >> (i * 4)) & 0xF; // 1010 = A-hex = 10-dec

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
        for(int b = readArray[r].bytesToRead * 2; b > 0; b = b - 4) {
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

    }
}

void psramArray_write(RamArrayConfig ramArray, uint32_t address, uint32_t bytesToWrite, uint32_t* buffer) {
}

void calculateReads(RamArrayConfig ramArray, uint32_t startingAddress, uint32_t bytesToRead, RamArrayRead* readArray, uint* requiredReads) {
}