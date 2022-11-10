/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "ff.h" /* Obtains integer types */
#include "diskio.h" /* Declarations of disk functions */
#include "f_util.h"
#include "my_debug.h"
#include "rtc.h"
#include "hw_config.h"

#include "psram_inline.h"
#include "internal_sd_card.h"
#include "pio_spi/pio_spi.h"

#define SD_CARD_RX_READ_DEBUG 0

#define REGISTER_SD_COMMAND 0x0 // 1 byte, r/w
#define REGISTER_SD_READ_SECTOR 0x1 // 4 bytes
#define REGISTER_SD_READ_SECTOR_COUNT 0x5 // 4 bytes
#define COMMAND_START    0xDE
#define COMMAND_START2   0xAD
#define COMMAND_FINISH   0xBE
#define COMMAND_FINISH2  0xEF
#define COMMAND_SD_READ  0x72 // literally the r char
#define COMMAND_SD_WRITE 0x77 // literally the w char
#define DISK_READ_BUFFER_SIZE 512

#define PRINT_BUFFER_AFTER_SEND 0

int PC64_MCU_ID = -1;

__volatile uint16_t pc64_uart_tx_buf[PC64_BASE_ADDRESS_LENGTH];
uint8_t diskReadBuffer[DISK_READ_BUFFER_SIZE];

__volatile uint16_t sd_sector_registers[4];
__volatile uint32_t sd_read_sector_start;
__volatile uint32_t sd_read_sector_count;
__volatile char sd_selected_rom_title[256];
__volatile bool sd_is_busy = true;

// Variables used to signal sd data send from mcu2 -> mcu1 and the data that needs to be sent
__volatile bool sendDataReady = false;
__volatile uint32_t sectorToSend = 0;
__volatile uint32_t numSectorsToSend = 0;

// sd_card_t *pSD; // Reference to the SD card object. Only valid on MCU2

void pc64_set_sd_read_sector(uint32_t sector) {
    #if SD_CARD_RX_READ_DEBUG == 1
        printf("set read sector = %ld", sector);
    #endif
    sd_read_sector_start = sector;
}

void pc64_set_sd_read_sector_part(int index, uint16_t value) {
    #if SD_CARD_RX_READ_DEBUG == 1
        printf("set read sector part %d = %d", index, value);
    #endif
    sd_sector_registers[index] = value;
    // Assume we have set the other sectors as since this is the final piece
    // We can set the sector we want to actually read
    if (index == 3) {
        // uint64_t s = ((uint64_t)(sd_sector_registers[0]) << 48 | ((uint64_t)sd_sector_registers[1]) << 32 | sd_sector_registers[2] << 16 | sd_sector_registers[3]);
        uint32_t s = (uint32_t)(sd_sector_registers[2] << 16 | sd_sector_registers[3]);
        pc64_set_sd_read_sector(s);
    }
}

void pc64_set_sd_read_sector_count(uint32_t count) {
    #if SD_CARD_RX_READ_DEBUG == 1
        printf("set sector count = %d", count);
    #endif
    sd_read_sector_count = count;
}

void pc64_set_sd_rom_selection(char* title) {
    for (int i = 0; i < 256; i++) {
        sd_selected_rom_title[i] = title[i];
    }
}

void pc64_send_sd_read_command(void) {
    uint32_t sector = sd_read_sector_start;
    uint32_t sectorCount = 1;

    uint8_t cmd[16] = {
        COMMAND_START,
        COMMAND_START2,
        COMMAND_SD_READ,
        // (uint8_t)((sector & 0xFF00000000000000) >> 56),
        // (uint8_t)((sector & 0x00FF000000000000) >> 48),
        // (uint8_t)((sector & 0x0000FF0000000000) >> 40),
        // (uint8_t)((sector & 0x000000FF00000000) >> 32),
        // (uint8_t)((sector & 0x00000000FF000000) >> 24),
        // (uint8_t)((sector & 0x0000000000FF0000) >> 16),
        // (uint8_t)((sector & 0x000000000000FF00) >> 8),        
        // (uint8_t) (sector  & 0x00000000000000FF),
        (uint8_t)(0),
        (uint8_t)(0),
        (uint8_t)(0),
        (uint8_t)(0),
        (uint8_t)((sector & 0xFF000000) >> 24),
        (uint8_t)((sector & 0x00FF0000) >> 16),
        (uint8_t)((sector & 0x0000FF00) >> 8),        
        (uint8_t) (sector & 0x000000FF),
        (uint8_t)((sectorCount & 0xFF000000) >> 24),
        (uint8_t)((sectorCount & 0x00FF0000) >> 16),
        (uint8_t)((sectorCount & 0x0000FF00) >> 8),
        (uint8_t) (sectorCount & 0x000000FF),
        COMMAND_FINISH
    };

    // Write all the commands
    pio_spi_write8(cmd, 16);
}

// MCU2 listens for MCU1 commands and will respond accordingly
int startIndex = 0;
bool mayHaveStart = false;
bool mayHaveFinish = false;
bool receivingData = false;
void process_mcu2_cmd_buffer(unsigned char* mcu2_cmd_buffer, int len) {
    // read
    int index = 0;
    int bufferIndex = 0;
    do {
        char ch = mcu2_cmd_buffer[index];
        
        // printf("%02x ", ch);
        // if (ch == 0xAA) {
        //     printf("\n");
        // }

        if (ch == COMMAND_START) {
            mayHaveStart = true;
        } else if (ch == COMMAND_START2 && mayHaveStart) {
            receivingData = true;
        } else if (ch == COMMAND_FINISH && receivingData) {
            mayHaveFinish = true;
            receivingData = false;
        } else if (receivingData && !mayHaveFinish) {
            mcu2_cmd_buffer[bufferIndex] = ch;
            bufferIndex++;
        }

        if (mayHaveFinish && !receivingData) {
            // end of command
            // process what was sent
            char command = mcu2_cmd_buffer[0];
            uint32_t sector_front =(mcu2_cmd_buffer[1] << 24) | (mcu2_cmd_buffer[2] << 16) | (mcu2_cmd_buffer[3] << 8) | mcu2_cmd_buffer[4];
            uint32_t sector_back = (mcu2_cmd_buffer[5] << 24) | (mcu2_cmd_buffer[6] << 16) | (mcu2_cmd_buffer[7] << 8) | mcu2_cmd_buffer[8];
            // uint64_t sector = ((uint64_t)sector_front) << 32 | sector_back;
            uint32_t sector = sector_back;
            volatile uint32_t sectorCount = (mcu2_cmd_buffer[9] << 24) | (mcu2_cmd_buffer[10] << 16) | (mcu2_cmd_buffer[11] << 8) | mcu2_cmd_buffer[12];

            if (command == COMMAND_SD_READ) {
                sectorToSend = sector;
                numSectorsToSend = 1;
                sendDataReady = true;
            } else {
                // not supported yet
                printf("\nUnknown command: %x\n", command);
            }

            bufferIndex = 0;
            mayHaveFinish = false;
            mayHaveStart = false;
            receivingData = false;
        }

        if (bufferIndex-1 == 14) {
            bufferIndex = 0;
            printf("\nError: Missing last byte\n");
        }

        index++;
    } while (index < len);
}

void send_data(uint32_t sector, uint32_t count) {
    DRESULT dr = disk_read(0, diskReadBuffer, (uint64_t)sector, 1);
        
    if (dr != RES_OK) {
        printf("Error reading disk: %d\n", dr);
    } else {
        // Send sector worth of data
        pio_spi_write8(diskReadBuffer, DISK_READ_BUFFER_SIZE);
    }
}

// MCU2 will send data once it has the information it needs
// void send_data(uint32_t sector, uint32_t sectorCount) {
//     //printf("Sending data. Sector: %d, Count: %d\n", sector, sectorCount);
//     int loopCount = 0;
//     do {
//         loopCount++;
//         DRESULT dr = disk_read(0, diskReadBuffer, (uint64_t)sector, 1);
        
//         if (dr != RES_OK) {
//             printf("Error reading disk: %d\n", dr);
//         } 
        
//         sectorCount--;

//         // Send sector worth of data
//         pio_spi_write8(diskReadBuffer, DISK_READ_BUFFER_SIZE);

//     // Repeat if we are reading more than 1 sector
//     } while(sectorCount > 1);

//     printf("Sent %d sectors starting at Sector %d\n", loopCount, sector);

//     #if PRINT_BUFFER_AFTER_SEND == 1
//         // printf("buffer for sector: %ld\n", sector);
//         // for (uint diskBufferIndex = 0; diskBufferIndex < DISK_READ_BUFFER_SIZE; diskBufferIndex++) {
//         //         printf("%02x ", diskReadBuffer[diskBufferIndex]);
//         //     }
//         // printf("\n");

//         uint8_t lastBufChar = 0;
//         for (int i = 0; i < 512; i++) {
//             uint8_t value = diskReadBuffer[i];
//             if (i % 2 == 1) {
//                 uint16_t value16 = value << 8 | lastBufChar;
//                 printf("%04x ", value16);
//             } else {
//                 lastBufChar = value;
//             }

//             if (i % 10 == 0 && i != 0) {
//                 printf("\n");
// 			}
//         }

//     #endif
// }

// void send_sd_card_data() {
//     // Reset send data flag
//     sendDataReady = false; 

//     // Send the data over uart back to MCU1 so the rom can read it
//     uint64_t sector = sectorToSend;
//     uint32_t numSectors = 1;
//     send_data(sector, numSectors);
// }

// SD mount helper function
static sd_card_t *sd_get_by_name(const char *const name) {
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name)) return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
// SD mount helper function
static FATFS *sd_get_fs_by_name(const char *name) {
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name)) return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

void mount_sd(void) {
    printf("Mounting SD Card\n");

    for (int i = 0; i < 512; i++) {
        
        if (i == 511) {
            diskReadBuffer[i] == 0xBB;
        } else if (i == 0) {
            diskReadBuffer[i] = 0xFF;
        } else {
            diskReadBuffer[i] = 0x01;
        }
    }

    // // See FatFs - Generic FAT Filesystem Module, "Application Interface",
	// // http://elm-chan.org/fsw/ff/00index_e.html
	// pSD = sd_get_by_num(0);
	// FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	// if (FR_OK != fr) {
	// 	panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
	// }

    // printf("SD Card mounted. Status: %d\n", fr);

    // Original code from test project. Might not need this more complicated version
    //const char *arg1 = strtok(NULL, " ");
    //if (!arg1) arg1 = sd_get_by_num(0)->pcName;
    const char *arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs) {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr) {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    if (pSD == NULL) {
        printf("Error getting sd card by name: %s\n", arg1);
    }
    pSD->mounted = true;
}