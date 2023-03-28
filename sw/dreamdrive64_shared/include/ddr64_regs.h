/**
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * Copyright (c) 2022 Konrad Beckmann
 * Copyright (c) 2022 Kaili Hill
 */

#pragma once

// DreamDrive64 Address space

// [READ/WRITE]: Scratch memory used for various functions
#define DDR64_BASE_ADDRESS_START     (0x1FFE0000)
#define DDR64_BASE_ADDRESS_LENGTH    (0x0001000)
#define DDR64_BASE_ADDRESS_END       (DDR64_BASE_ADDRESS_START + DDR64_BASE_ADDRESS_LENGTH - 1)

// [READ/WRITE]: Command address space. See register definitions below for details.
#define DDR64_CIBASE_ADDRESS_START   (DDR64_BASE_ADDRESS_END + 1)
#define DDR64_CIBASE_ADDRESS_LENGTH  (0x00000800)
#define DDR64_CIBASE_ADDRESS_END     (DDR64_CIBASE_ADDRESS_START + DDR64_CIBASE_ADDRESS_LENGTH - 1)

// [Read]: Returns DDR64_MAGIC
#define DDR64_REGISTER_MAGIC         (0x00000000)
#define DDR64_MAGIC                  (0xD9D96400)

// [WRITE]: Write number of bytes to print from TX buffer
#define DDR64_REGISTER_UART_TX       (0x00000004)

// [WRITE]: Set the random seed to a 32-bit value
#define DDR64_REGISTER_RAND_SEED     (0x00000008)

/* *** SD CARD *** */
// [READ]: Signals pico to start data read from SD Card
#define DDR64_COMMAND_SD_READ       (DDR64_REGISTER_RAND_SEED + 0x4)

// [READ]: Load selected rom into memory and boot, 
#define DDR64_COMMAND_SD_ROM_SELECT (DDR64_COMMAND_SD_READ + 0x4)

// [READ] 1 while sd card is busy, 0 once the CI is free
#define DDR64_REGISTER_SD_BUSY (DDR64_COMMAND_SD_ROM_SELECT + 0x4)

// [WRITE] Sector to read from SD Card, 8 bytes
#define DDR64_REGISTER_SD_READ_SECTOR0 (DDR64_REGISTER_SD_BUSY + 0x4)
#define DDR64_REGISTER_SD_READ_SECTOR1 (DDR64_REGISTER_SD_READ_SECTOR0 + 0x4)

// [WRITE] number of sectors to read from the sd card, 4 bytes
#define DDR64_REGISTER_SD_READ_NUM_SECTORS (DDR64_REGISTER_SD_READ_SECTOR1 + 0x4)

// [WRITE] write the selected file name that should be loaded into memory
// 255 bytes
#define DDR64_REGISTER_SD_SELECT_ROM (DDR64_REGISTER_SD_READ_NUM_SECTORS + 0x4)

// [WRITE] Register to define the cic type and save type.
// 0xFF00 == Cic
// 0x00FF == save
#define DDR64_REGISTER_SELECTED_ROM_META (DDR64_REGISTER_SD_SELECT_ROM + 0x4)

