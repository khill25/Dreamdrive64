/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "errno.h"

#include "pico/stdlib.h"
#include "hardware/structs/systick.h"

#include "ff.h" /* Obtains integer types */
#include "diskio.h" /* Declarations of disk functions */
#include "f_util.h"
#include "my_debug.h"
#include "rtc.h"
#include "hw_config.h"

#include "qspi_helper.h"
#include "internal_sd_card.h"
#include "psram.h"
#include "ringbuf.h"
#include "joybus/joybus.h"

#include "utils.h"
#include "FreeRTOS.h"
#include "task.h"

#define SD_CARD_RX_READ_DEBUG 0

#define REGISTER_SD_COMMAND 0x0 // 1 byte, r/w
#define REGISTER_SD_READ_SECTOR 0x1 // 4 bytes
#define REGISTER_SD_READ_SECTOR_COUNT 0x5 // 4 bytes
#define COMMAND_START    0xDE
#define COMMAND_START2   0xAD
#define COMMAND_SD_READ  0x72 // literally the r char
#define COMMAND_SD_WRITE 0x77 // literally the w char
#define COMMAND_LOAD_ROM 0x6C // literally the l char
#define COMMAND_ROM_LOADED 0xC6 // inverse of the load rom command
#define COMMAND_VERIFY_ROM_DATA 0xF1 // Command to check the data sent
#define COMMAND_BACKUP_EEPROM  (0xBE)
#define COMMAND_LOAD_BACKUP_EEPROM  (0xEB)
#define COMMAND_SET_EEPROM_TYPE  (0xE7)
#define COMMAND_SET_ROM_META_INFO  (0xA1)
#define DISK_READ_BUFFER_SIZE 512

#define DEBUG_MCU2_PSRAM_SANITY_TEST 0
#define DEBUG_MCU2_PRINT 0
#define PRINT_BUFFER_AFTER_SEND 0
#define MCU1_ECHO_RECEIVED_DATA 0
#define MCU2_PRINT_UART 0

int DDR64_MCU_ID = -1;

volatile int bufferIndex = 0; // Used on MCU1 to track where to put the next byte received from MCU2
uint8_t lastBufferValue = 0;
int bufferByteIndex = 0;
volatile uint16_t ddr64_uart_tx_buf[DDR64_BASE_ADDRESS_LENGTH];

volatile uint32_t sd_sector_registers[4];
volatile uint32_t sd_sector_count_registers[2];
volatile uint32_t sd_read_sector_start;
volatile uint32_t sd_read_sector_count;
char sd_selected_rom_title[256]; // TODO this buffer will need to be larger if the rom is in a sub directory
volatile uint32_t sd_selected_title_length_registers[2];
volatile uint32_t sd_selected_title_length = 0;
volatile bool sd_is_busy = false;
volatile uint32_t selected_rom_metadata_register;

volatile int selected_rom_save_type = 0;    // Default to no save type
volatile int selected_rom_cic = 2;          // 6102
volatile int selected_rom_cic_region = -1;  // Default

// Variables used for signalling sd data send 
volatile bool waitingForRomLoad = false;
volatile bool sendDataReady = false;
volatile uint32_t sectorToSendRegisters[2];
volatile uint32_t numSectorsToSend = 0;
volatile bool startRomLoad = false;
volatile bool romLoading = false;
volatile uint16_t eeprom_numBytesToBackup = 0;
volatile bool start_saveEeepromData = false;
volatile bool start_loadEeepromData = false;
volatile bool is_verifying_rom_data_from_mcu1 = false;
volatile uint32_t verifyDataTime = 0;

void save_eeprom_to_sd(FIL* eepromFile);
void load_eeprom_from_sd(FIL* eepromFile);

void ddr64_set_sd_read_sector_part(int index, uint32_t value) {
    #if SD_CARD_RX_READ_DEBUG == 1
        printf("set read sector part %d = %d", index, value);
    #endif
    sd_sector_registers[index] = value;
}

void ddr64_set_sd_read_sector_count(int index, uint32_t count) {
    sd_sector_count_registers[index] = count; 
}

void ddr64_set_sd_rom_selection_length_register(uint32_t value, int index) {
    sd_selected_title_length_registers[index] = value;
}

void ddr64_set_sd_rom_selection(char* titleBuffer, uint32_t len) {
    sd_selected_title_length = len >> 16;
    strncpy(sd_selected_rom_title, titleBuffer, sd_selected_title_length);
}

void ddr64_set_rom_meta_data(uint32_t value, int index) {
    if (index == 0) {
        selected_rom_metadata_register = value;
    } else {
        selected_rom_metadata_register |= value;
    }
}

void ddr64_send_sd_read_command(void) {
    // Block cart while waiting for data
    sd_is_busy = true;
    sendDataReady = false;
    bufferIndex = 0;
    bufferByteIndex = 0;
    uint32_t sectorCount = 1;

    // Signal start
    uart_tx_program_putc(COMMAND_START);
    uart_tx_program_putc(COMMAND_START2);

    // command
    uart_tx_program_putc(COMMAND_SD_READ);

    // 12 bytes to read
    uart_tx_program_putc(0);
    uart_tx_program_putc(12);

    // sector (top bytes)
    uart_tx_program_putc((char)(sd_sector_registers[0] >> 24));
    uart_tx_program_putc((char)(sd_sector_registers[0] >> 16));
    uart_tx_program_putc((char)(sd_sector_registers[1] >> 24));
    uart_tx_program_putc((char)(sd_sector_registers[1] >> 16));

    // sector (bottom bytes)
    uart_tx_program_putc((char)(sd_sector_registers[2] >> 24));
    uart_tx_program_putc((char)(sd_sector_registers[2] >> 16));
    uart_tx_program_putc((char)(sd_sector_registers[3] >> 24));
    uart_tx_program_putc((char)(sd_sector_registers[3] >> 16));

    // num sectors
    uart_tx_program_putc((char)((sectorCount & 0xFF000000) >> 24));
    uart_tx_program_putc((char)((sectorCount & 0x00FF0000) >> 16));
    uart_tx_program_putc((char)((sectorCount & 0x0000FF00) >> 8));
    uart_tx_program_putc((char) (sectorCount & 0x000000FF));
}

// Send command from MCU1 to MCU2 to start loading a rom
void ddr64_send_load_new_rom_command() {
    // Block cart while waiting for data
    sd_is_busy = true;
    sendDataReady = false;
    romLoading = true;
    bufferIndex = 0;
    bufferByteIndex = 0;

    // send metadata first
    uart_tx_program_putc(COMMAND_START);
    uart_tx_program_putc(COMMAND_START2);

    // Command
    uart_tx_program_putc(COMMAND_SET_ROM_META_INFO);

    // Sending 4 bytes
    uart_tx_program_putc(0);
    uart_tx_program_putc(4);

    // Data
    uart_tx_program_putc(selected_rom_metadata_register >> 24);
    uart_tx_program_putc(selected_rom_metadata_register >> 16);
    uart_tx_program_putc(selected_rom_metadata_register >> 8);
    uart_tx_program_putc(selected_rom_metadata_register);

    // Now send rom to load info
    uint32_t rom_title_len = strlen(sd_selected_rom_title);

    // Signal start
    uart_tx_program_putc(COMMAND_START);
    uart_tx_program_putc(COMMAND_START2);

    // command
    uart_tx_program_putc(COMMAND_LOAD_ROM);

    // Number of bytes based on the length of the rom title
    uart_tx_program_putc(rom_title_len >> 8);
    uart_tx_program_putc(rom_title_len);

    for(int i = 0; i < rom_title_len; i++) {
        uart_tx_program_putc(sd_selected_rom_title[i]);
    }
}

volatile int current_verify_test_freq_khz = 210000;
volatile uint32_t verify_rom_data_total_bytes_to_read = PSRAM_CHIP_CAPACITY_BYTES * 8;
void verify_rom_data_helper() {
    volatile uint32_t *ptr_32 = (volatile uint32_t *)0x13000000;
    volatile uint16_t *ptr_16 = (volatile uint16_t *)0x13000000;
	volatile uint8_t *ptr_8 = (volatile uint8_t *)0x13000000;

	for(int i = 1; i <= 8; i++) {
		psram_set_cs(i);
        uint32_t numBytesToRead = PSRAM_CHIP_CAPACITY_BYTES;//i == 1 ? PSRAM_CHIP_CAPACITY_BYTES : (4 * 1024 * 1024);
		// Read 512 bytes of data at a time into a buffer, loop until we have read 8MB
		// for(int k = 0; k < PSRAM_CHIP_CAPACITY_BYTES; k+=512) {
        for(int k = 0; k < numBytesToRead; k+=512) {
        // while(1) {
			// Command instruction
			uart_tx_program_putc(COMMAND_START);
    		uart_tx_program_putc(COMMAND_START2);
			uart_tx_program_putc(COMMAND_VERIFY_ROM_DATA);

            // Sending 512 bytes of data
            uint16_t bytesToSend = 512;
			uart_tx_program_putc(bytesToSend >> 8);
            uart_tx_program_putc(bytesToSend);

			// Read one byte from psram and send it to mcu2
			// Read 512 bytes at a time
			for(int j = 0; j < 256; j++) {
				volatile uint32_t addr = (k/2) + j;
                volatile uint16_t word = ptr_16[addr];
                // volatile uint8_t word = ptr_8[addr];
                // volatile uint32_t word = ptr_32[addr];

                // printf("[%08x]: %04x\n", addr*2, word);
                while (!uart_tx_program_is_writable()) {
                    tight_loop_contents();
                }
				uart_tx_program_putc((char)(word));

                while (!uart_tx_program_is_writable()) {
                    tight_loop_contents();
                }
                uart_tx_program_putc((char)(word >> 8));
			}

            // Sleep to allow time to verify data 
            // so mcu2 doesn't get overwhelemed?
			sleep_ms(3);
		}

        sleep_ms(50);
	}
}
void verify_rom_data() {
    while(1) {
        // ssi_hw->ssienr = 0;
        // if (current_verify_test_freq_khz < 200000) {
        //     ssi_hw->rx_sample_dly = 1;
        // } else if (current_verify_test_freq_khz >= 170000 && current_verify_test_freq_khz < 210000) {
        //     ssi_hw->rx_sample_dly = 2;
        // } else {
        //     ssi_hw->rx_sample_dly = 3;
        // }
        // ssi_hw->ssienr = 1;

	    verify_rom_data_helper();
        // current_verify_test_freq_khz += 10000;
        // bool clockWasSet = set_sys_clock_khz(current_verify_test_freq_khz, false);

        sleep_ms(1000);
    }
}

FIL verify_data_fil;
const char* verify_data_filename = "Resident Evil 2 (USA) (Rev 1).z64";//"GoldenEye 007 (U) [!].z64"
void mcu2_setup_verify_rom_data() {
    sd_card_t *pSD = sd_get_by_num(0);
	FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	if (FR_OK != fr) {
		panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
	}

	printf("\n\n---- read /%s -----\n", verify_data_filename);

	fr = f_open(&verify_data_fil, verify_data_filename, FA_OPEN_EXISTING | FA_READ);
	if (FR_OK != fr && FR_EXIST != fr) {
		panic("f_open(%s) error: %s (%d)\n", verify_data_filename, FRESULT_str(fr), fr);
	}

	FILINFO filinfo;
	fr = f_stat(verify_data_filename, &filinfo);
	printf("%s [size=%llu]\n", filinfo.fname, filinfo.fsize);
}

const uint32_t value_to_report_in = (1 * 1024);
volatile uint32_t verify_data_total = 0;
volatile int verify_data_currentChip = START_ROM_LOAD_CHIP_INDEX;
volatile int verify_data_whole_array_error_count = 0;
volatile int verify_data_chipErrorCount[9] = {0};
uint32_t verify_data_error_addresses[16] = {0};
volatile uint32_t numCallsToVerifyFunction = 0;
void mcu2_verify_sent_rom_data() {
    FRESULT fr;
    char buf[512];
    int len = 0;
    numCallsToVerifyFunction++;

    fr = f_read(&verify_data_fil, buf, sizeof(buf), &len);
    uint32_t addr = verify_data_total - ((verify_data_currentChip - START_ROM_LOAD_CHIP_INDEX) * PSRAM_CHIP_CAPACITY_BYTES);
    
    if(len == 0) {
        len = 512;
    }

    volatile uint16_t* buf_16 = (volatile uint16_t*)buf;
    volatile uint16_t *ptr_16 = (volatile uint16_t *)ddr64_uart_tx_buf;
    
    for(int i = 0; i < len/2; i++) {
        uint16_t word = ptr_16[i];
        uint16_t b = buf_16[i];
        // if (i < 32 && numCallsToVerifyFunction <= 1) {
        //     printf("[%08x]: %04x!=%04x\n", addr+(i*2), b, word);
        // }
        if(b != word) {
            verify_data_whole_array_error_count++;
            if (verify_data_whole_array_error_count < 4) {
                // verify_data_error_addresses[verify_data_whole_array_error_count] = addr+(i*2);
                printf("[%08x]: %04x!=%04x\n", addr+(i*2), b, word);
            }
            verify_data_chipErrorCount[verify_data_currentChip]++;
        }
    }

    verify_data_total += len;

    int newChip = psram_addr_to_chip(verify_data_total);
    if (newChip != verify_data_currentChip) {
        if (verify_data_chipErrorCount[verify_data_currentChip] > 0) {
            printf("[%d]x = %d | ", verify_data_currentChip, verify_data_chipErrorCount[verify_data_currentChip]);
        } else {
            printf("[%d].| ", verify_data_currentChip);
        }

        verify_data_currentChip = newChip;
    }

    // if (numCallsToVerifyFunction % 512 == 0 && numCallsToVerifyFunction > 1) {
    //     printf(".");
    // }

    // If we have read the entirety of the file, print out the stats
    if (verify_data_total >= verify_rom_data_total_bytes_to_read) {
        printf("\n");
        uint32_t totalVerifyTime = time_us_32() - verifyDataTime;
        for(int i = 1; i <= 8; i++) {
            //printf("%d errors on chip %d\n", verify_data_chipErrorCount[i], i);
            verify_data_chipErrorCount[i] = 0; // reset error count
        }
        printf("Found %d errors\n\n", verify_data_whole_array_error_count);
        // printf("Finished!\n");

        // More data reset
        f_rewind(&verify_data_fil);
        verify_data_total = 0; 
        verify_data_whole_array_error_count = 0;
        numCallsToVerifyFunction = 0;

        // increase processor clock
        // current_verify_test_freq_khz += 10000;
        // bool clockWasSet = set_sys_clock_khz(current_verify_test_freq_khz, false);
        // printf("Setting new clock: %dKHz\n", current_verify_test_freq_khz);

        // if(current_verify_test_freq_khz >= 250001) {
        //     printf("Finished all frequencies for 2x divider.\n");
        //     while(1) {tight_loop_contents();}
        // }
    }
}

void extract_metadata_and_send_save_info(char* buf, FIL* fil) {
    printf("Rom serial: %c%c%c%c\n", buf[0x3B], buf[0x3C], buf[0x3D], buf[0x3E]);

    int saveType = selected_rom_metadata_register & 0x000000FF;
    selected_rom_cic = selected_rom_metadata_register >> 16;

    printf("meta register: %08x\n", selected_rom_metadata_register);
    printf("CIC: %d, saveType: %d, country: %c\n", selected_rom_cic, saveType, buf[0x3E]);
    printf("Sending eeprom info to mcu1...\n");

    uart_tx_program_putc(COMMAND_START);
    uart_tx_program_putc(COMMAND_START2);
    uart_tx_program_putc(COMMAND_SET_EEPROM_TYPE);
    uart_tx_program_putc(0);
    uart_tx_program_putc(2);
    
    if(saveType == 3) {
        eeprom_type = EEPROM_TYPE_4K;
        uart_tx_program_putc((uint8_t)(EEPROM_TYPE_4K >> 8));
        uart_tx_program_putc((uint8_t)(EEPROM_TYPE_4K));
    } else if (saveType == 4) {
        eeprom_type = EEPROM_TYPE_16K;
        uart_tx_program_putc((uint8_t)(EEPROM_TYPE_16K >> 8));
        uart_tx_program_putc((uint8_t)(EEPROM_TYPE_16K));
    } else {
        // Don't use eeprom
        eeprom_type = 0;
        uart_tx_program_putc(0);
        uart_tx_program_putc(0);
    }

    if (saveType == 3 || saveType == 4) {
        // Busy wait for a few cycles then send eeprom data
        for(int i = 0; i < 10000; i++) { tight_loop_contents(); }

        // Send the eeprom save data
        load_eeprom_from_sd(fil);
        printf("Finished sending eeprom data to mcu1!\n");

        // for(int i = 0; i < 10000; i++) { tight_loop_contents(); }
    }
}

void load_selected_rom() {
    printf("Loading '%s'...\n", sd_selected_rom_title);
    load_new_rom(sd_selected_rom_title);
}

void load_new_rom(char* filename) {
    sd_is_busy = true;
    char buf[512 * 4];
    printf("Mounting sd card...\n");
    sd_card_t *pSD = sd_get_by_num(0);
	FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	if (FR_OK != fr) {
		panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
	}

    sleep_ms(10);
    printf("Mounted!\n");

	FIL fil;

	printf("\n\n---- read /%s -----\n", filename);

    printf("Open file...\n");
	fr = f_open(&fil, filename, FA_OPEN_EXISTING | FA_READ);
	if (FR_OK != fr && FR_EXIST != fr) {
		panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
	}

	FILINFO filinfo;
	fr = f_stat(filename, &filinfo);
	printf("%s [size=%llu]\n", filinfo.fname, filinfo.fsize);

    int currentPSRAMChip = START_ROM_LOAD_CHIP_INDEX;
    qspi_enable_spi(4, currentPSRAMChip);

    printf("Writing to psram...\n");
	int len = 0;
	int total = 0;
    volatile bool isFirstRead = true;
	uint64_t t0 = to_us_since_boot(get_absolute_time());
	do {
        fr = f_read(&fil, buf, sizeof(buf), &len);
        uint32_t addr = total - ((currentPSRAMChip - START_ROM_LOAD_CHIP_INDEX) * PSRAM_CHIP_CAPACITY_BYTES);
        
        // Write data to the psram chips
        qspi_spi_write_buf(addr, buf, len);
		
        total += len;

        // Once we have read the first chunk of bytes this includes the rom header
        // With this info we can lookup save and cic info for this rom!
        // Load any saved eeprom to mcu1.
        if (isFirstRead) {
            isFirstRead = false;

            fr = f_close(&fil);

            printf("Finding rom info...\n");
            extract_metadata_and_send_save_info(buf, &fil);
            printf("Resuming rom load...\n");

            fr = f_open(&fil, filename, FA_OPEN_EXISTING | FA_READ);
            f_lseek(&fil, len);
        }

        int newChip = psram_addr_to_chip(total);
        if (newChip != currentPSRAMChip && newChip <= MAX_MEMORY_ARRAY_CHIP_INDEX) {
            printf("Changing memory array chip. Was: %d, now: %d\n", currentPSRAMChip, newChip);
            printf("Total bytes: %d. Bytes remaining = %ld\n", total, (filinfo.fsize - total));
            currentPSRAMChip = newChip;
            psram_set_cs(currentPSRAMChip); // Switch the PSRAM chip
        }

	} while (len > 0);

	uint64_t t1 = to_us_since_boot(get_absolute_time());
	uint32_t delta = (t1 - t0) / 1000;
	uint32_t kBps = (uint32_t) ((float)(total / 1024.0f) / (float)(delta / 1000.0f));

	printf("Read %d bytes and programmed PSRAM in %d ms (%d kB/s)\n\n\n", total, delta, kBps);

	fr = f_close(&fil);
	if (FR_OK != fr) {
		printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
	}
	printf("---- read file done -----\n\n\n");

    #if DEBUG_MCU2_PSRAM_SANITY_TEST == 1
    // Enter quad mode and enable qspi to do a data sanity check
    qspi_enable_qspi(START_ROM_LOAD_CHIP_INDEX, MAX_MEMORY_ARRAY_CHIP_INDEX);

    // Now enable xip and try to read
    for (int o = 1; o <= 8; o++) {
        psram_set_cs(o); // Set the PSRAM chip to use
        sleep_ms(50);
        printf("\n\nCheck data from U%u...\n", o);
        volatile uint32_t *ptr = (volatile uint32_t *)0x13000000;
        for (int i = 0; i < 16; i++) {
            volatile uint32_t word = ptr[i];
            if (i < 4) {
                printf("PSRAM-MCU2[%08x]: %08x\n", i * 4, word);
            }
        }
        
        // exit quad mode once finished with read
        qspi_qspi_exit_quad_mode();
        sleep_ms(100);
    }
    #endif

    // Now turn off the ssi hardware so mcu1 can use it
    qspi_disable();

    printf("Rom Loaded, MCU2 qspi: OFF, sending mcu1 rom loaded command\n");

    // Let MCU1 know that we are finished
    uart_tx_program_putc(COMMAND_START);
    uart_tx_program_putc(COMMAND_START2);
    uart_tx_program_putc(COMMAND_ROM_LOADED);
    // Zero bytes to read!
    uart_tx_program_putc(0x00);
    uart_tx_program_putc(0x00);

    // TODO/OFI
    // Send mcu1 the size of the loaded rom
    // and any other relevant metadata
}

// MCU listens for other MCU commands and will respond accordingly
int startIndex = 0;
bool mayHaveStart = false;
bool mayHaveFinish = false;
bool receivingData = false;
bool isReadingCommandHeader = false;
uint8_t command_headerBufferIndex = 0;
uint16_t command_numBytesToRead = 0;
bool command_processBuffer = false;

#define COMMAND_HEADER_LENGTH 3
//COMMAND, NUM_BYTES_HIGH, NUM_BYTES_LOW
uint8_t commandHeaderBuffer[COMMAND_HEADER_LENGTH]; 

unsigned char mcu2_cmd_buffer[64]; // TODO rename
int echoIndex = 0;
void mcu1_process_rx_buffer() {
    while (rx_uart_buffer_has_data()) {
        uint8_t value = rx_uart_buffer_get();
        
        #if MCU1_ECHO_RECEIVED_DATA == 1
        uart_tx_program_putc(value);
        #endif

        if (romLoading) {
            if (receivingData) {
                // Special case to send bytes directly into the eeprom array
                if (commandHeaderBuffer[0] == COMMAND_LOAD_BACKUP_EEPROM) {
                    eeprom[bufferIndex] = value;
                } else {
                    ((uint8_t*)(ddr64_uart_tx_buf))[bufferIndex] = value;
                }

                bufferIndex++;
                if (bufferIndex >= command_numBytesToRead) {
                    command_processBuffer = true;
                    bufferIndex = 0;
                }
                
            } else if (isReadingCommandHeader) {
                commandHeaderBuffer[command_headerBufferIndex++] = value;

                if (command_headerBufferIndex >= COMMAND_HEADER_LENGTH) {
                    command_numBytesToRead = (commandHeaderBuffer[1] << 8) | commandHeaderBuffer[2];
                    isReadingCommandHeader = false;
                    command_headerBufferIndex = 0;

                    // Special case for num bytes to read 0, skip right to processing command
                    if (command_numBytesToRead == 0) {
                        command_processBuffer = true;
                        bufferIndex = 0;
                    } else {
                        receivingData = true;
                    }
                }
            } else if (value == COMMAND_START && !receivingData) {
                mayHaveStart = true;
            } else if (value == COMMAND_START2 && mayHaveStart && !receivingData) {
                isReadingCommandHeader = true;
            }

            if (command_processBuffer) {
                command_processBuffer = false;
                // process what was sent
                uint8_t* buffer = (uint8_t*)ddr64_uart_tx_buf; // cast to char array
                char command = commandHeaderBuffer[0];

                if (command == COMMAND_SET_EEPROM_TYPE) {
                    eeprom_type = (buffer[0] << 8 | buffer[1]);

                } else if (command == COMMAND_LOAD_BACKUP_EEPROM) {
                    // Already pushed these bits into the eeprom array

                } else if (command == COMMAND_ROM_LOADED) {
                    romLoading = false; // signal that the rom is finished loading
                    sendDataReady = true;
                }

                // Reset state and empty the command header buffer
                commandHeaderBuffer[0] = 0;
                commandHeaderBuffer[1] = 0;
                commandHeaderBuffer[2] = 0;
                bufferIndex = 0;
                mayHaveFinish = false;
                mayHaveStart = false;
                receivingData = false;
            }
        } else {
            // Combine two char values into a 16 bit value
            // Only increment bufferIndex when adding a value
            // else, store the ch into the holding field
            if (bufferByteIndex % 2 == 1) {
                uint16_t value16 = lastBufferValue << 8 | value;
                ddr64_uart_tx_buf[bufferIndex] = value16;
                bufferIndex += 1;
            } else {
                lastBufferValue = value;
            }

            bufferByteIndex++;

            if (bufferByteIndex >= SD_CARD_SECTOR_SIZE) {
                sendDataReady = true;
                break;
            }

            if (bufferByteIndex >= 512) {
                sendDataReady = true;
            }
        }
    }
}

void mcu2_process_rx_buffer() {
    while (rx_uart_buffer_has_data()) {
        char ch = rx_uart_buffer_get();
        
        #if MCU2_PRINT_UART == 1
        printf("%02x ", ch);
        #endif

        if (receivingData) {
            ((uint8_t*)(ddr64_uart_tx_buf))[bufferIndex] = ch;
            bufferIndex++;

            if (bufferIndex >= command_numBytesToRead) {
                command_processBuffer = true;
                bufferIndex = 0;
            }
        } else if (isReadingCommandHeader) {
            commandHeaderBuffer[command_headerBufferIndex++] = ch;

            if (command_headerBufferIndex >= COMMAND_HEADER_LENGTH) {
                command_numBytesToRead = (commandHeaderBuffer[1] << 8) | commandHeaderBuffer[2];
                isReadingCommandHeader = false;
                receivingData = true;
                command_headerBufferIndex = 0;
            }

            #if MCU2_PRINT_UART == 1
            printf("\n");
            #endif
            
        } else if (ch == COMMAND_START && !receivingData) {
            mayHaveStart = true;
        } else if (ch == COMMAND_START2 && mayHaveStart && !receivingData) {
            isReadingCommandHeader = true;
        }

        if (command_processBuffer) {
            command_processBuffer = false;
            // process what was sent
            uint8_t* buffer = (uint8_t*)ddr64_uart_tx_buf; // cast to char array
            char command = commandHeaderBuffer[0];

            if (command == COMMAND_SD_READ) {
                uint32_t sector_front =(buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
                uint32_t sector_back = (buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7];
                volatile uint32_t sectorCount = (buffer[8] << 24) | (buffer[9] << 16) | (buffer[10] << 8) | buffer[11];
                sectorToSendRegisters[0] = sector_front;
                sectorToSendRegisters[1] = sector_back;
                numSectorsToSend = 1;
                sendDataReady = true;
                
            } else if (command == COMMAND_LOAD_ROM) {
                sprintf(sd_selected_rom_title, "%s", buffer);
                startRomLoad = true;
                #if DEBUG_MCU2_PRINT == 1
                printf("nbtr: %u\n", command_numBytesToRead);
                #endif

            } else if (command == COMMAND_BACKUP_EEPROM) {
                eeprom_numBytesToBackup = command_numBytesToRead;
                start_saveEeepromData = true;
                #if DEBUG_MCU2_PRINT == 1
                printf("eeprom nbtr: %u\n", command_numBytesToRead);
                #endif

            } else if (command == COMMAND_VERIFY_ROM_DATA) {
                is_verifying_rom_data_from_mcu1 = true;

            } else if (command == COMMAND_SET_ROM_META_INFO) {
                printf("%02x %02x %02x %02x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
                selected_rom_metadata_register = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]);
            }
            else {
                // not supported yet
                printf("\nUnknown command: %x\n", command);
            }

            bufferIndex = 0;
            mayHaveFinish = false;
            mayHaveStart = false;
            receivingData = false;
            command_numBytesToRead = 0;

            #if MCU2_PRINT_UART == 1
                echoIndex = 0;
                printf("\n");
            #endif
        } else {
            #if MCU2_PRINT_UART == 1
            echoIndex++;
            if (echoIndex >= 32) {
                printf("\n");
                echoIndex = 0;
            }
            #endif
        }
    }
}

void extract_filename_from_possible_filepath(char* filepath, char* filename) {
    int len = strlen(filepath);
    int slashIndex = -1;
    for(int i = len; i >= 0; i--) {
        if(filepath[i] == '/') {
            slashIndex = i;
            break;
        }
    }

    if (slashIndex >= 0) {
        printf("Found a slash in the file path! Getting filename: ");
        sprintf(filename, "%s", filepath+slashIndex+1);
        printf("%s\n", filename);
    } else {
        printf("No slash in the file path: \n");
        sprintf(filename, "%s", filepath);
        printf("%s\n", filename);
    }
}

void start_eeprom_sd_save() {
    FIL eepromSave;
    save_eeprom_to_sd(&eepromSave);
}

void save_eeprom_to_sd(FIL* eepromFile) {
    printf("Saving eeprom data...\n");
    // Open or create file for currently loaded rom
    char* eepromSaveFilename = malloc(256 + 5); // 256 for max length rom filename + 4 for '.eep' and a terminating character
    char* extractedFilename = malloc(256 + 5);

    extract_filename_from_possible_filepath(sd_selected_rom_title, extractedFilename);
    sprintf(eepromSaveFilename, "0:/ddr_firmware/n64/eeprom/%s.eep", extractedFilename);
    free(extractedFilename);

    FRESULT fr = f_open(eepromFile, eepromSaveFilename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("'%s' Cannot be opened. Error: %u\n", eepromSaveFilename, fr);
        printf("Aborting eeprom save :(\n");
        free(eepromSaveFilename);
        return;
    }

    free(eepromSaveFilename);

    printf("Writing %u bytes\n", eeprom_numBytesToBackup);

    uint8_t* buf = (uint8_t*)ddr64_uart_tx_buf;
    uint numWritten = 0;
    f_write(eepromFile, buf, eeprom_numBytesToBackup, &numWritten);
    f_close(eepromFile);
    if (numWritten != eeprom_numBytesToBackup) {
        printf("Error saving eeprom. Wrote %d but expected %u\n", numWritten, eeprom_numBytesToBackup);
    } else {
        printf("Eeprom saved to %s\n", eepromSaveFilename);
    }
}

void load_eeprom_from_sd(FIL* eepromFile) {
    start_loadEeepromData = false;

    // Open or create file for currently loaded rom
    char* eepromSaveFilename = malloc(256 + 5); // 256 for max length rom filename + 4 for '.eep' and a terminating character
    char* tempFilename = malloc(256 + 5);

    extract_filename_from_possible_filepath(sd_selected_rom_title, tempFilename);
    sprintf(eepromSaveFilename, "0:/ddr_firmware/n64/eeprom/%s.eep", tempFilename);
    free(tempFilename);
    
    FRESULT fr = f_open(eepromFile, eepromSaveFilename, FA_READ);
    if (FR_OK != fr && FR_EXIST != fr) {
        printf("'%s' file not found. Error: %u\n", eepromSaveFilename, fr);
        free(eepromSaveFilename);
        return;
    }

    free(eepromSaveFilename);

    printf("EEPROM file opened. Reading bytes...\n");
    uint16_t numBytesToSend = eeprom_type == EEPROM_TYPE_4K ? 512 : 2048;
    printf("EEPROM is %u bytes\n", numBytesToSend);
    uint8_t* buf = (uint8_t*)ddr64_uart_tx_buf;
    uint numRead = 0;

    fr = f_read(eepromFile, buf, numBytesToSend, &numRead);
    if(fr != FR_OK) {
        printf("Error reading from eepromfile. Error: %u\n", fr);
        f_close(eepromFile);
        return;
    }

    fr = f_close(eepromFile);
    if (numRead != numBytesToSend) {
        printf("Error reading eeprom. Read %d but expected %u\n", numRead, numBytesToSend);
        return;
    }

    printf("Sending %u bytes\n", numBytesToSend);
    uart_tx_program_putc(COMMAND_START);
    uart_tx_program_putc(COMMAND_START2);
    uart_tx_program_putc(COMMAND_LOAD_BACKUP_EEPROM);
    uart_tx_program_putc((uint8_t)(numBytesToSend >> 8));
    uart_tx_program_putc((uint8_t)(numBytesToSend));

    for(int i = 0; i < numBytesToSend; i++) {
        while (!uart_tx_program_is_writable()) {
            tight_loop_contents();
        }
        uart_tx_program_putc(buf[i]);
    }
}

BYTE diskReadBuffer[DISK_READ_BUFFER_SIZE];
// MCU2 will send data once it has the information it needs
void send_data(uint32_t sectorCount) {
    uint64_t sectorFront = sectorToSendRegisters[0];
    uint64_t sector = (sectorFront << 32) | sectorToSendRegisters[1];
    #if DEBUG_MCU2_PRINT == 1
    printf("Count: %u, Sector: %llu\n", sectorCount, sector);
    #endif
    int loopCount = 0;
    uint32_t startTime = time_us_32();
    do {
        loopCount++;

        DRESULT dr = disk_read(0, diskReadBuffer, (uint64_t)sector, 1);        
        if (dr != RES_OK) {
            printf("Error reading disk: %d\n", dr);
        } 
        
        sectorCount--;

        // Send sector worth of data
        for (int diskBufferIndex = 0; diskBufferIndex < DISK_READ_BUFFER_SIZE; diskBufferIndex++) {
            // wait until uart is writable
            while (!uart_tx_program_is_writable()) {
                tight_loop_contents();
            }

            uart_tx_program_putc(diskReadBuffer[diskBufferIndex]);
        }

    // Repeat if we are reading more than 1 sector
    } while(sectorCount > 1);
    
    #if PRINT_BUFFER_AFTER_SEND == 1
    printf("buffer for sector: %ld\n", sector);
    for (uint diskBufferIndex = 0; diskBufferIndex < DISK_READ_BUFFER_SIZE; diskBufferIndex++) {
        if (diskBufferIndex % 16 == 0) {
            printf("\n%08x: ", diskBufferIndex);
        }
        printf("%02x ", diskReadBuffer[diskBufferIndex]);
    }
    printf("\n");
    #endif
}

void send_sd_card_data() {
    // Reset send data flag
    sendDataReady = false; 

    // Send the data over uart back to MCU1 so the rom can read it
    // Sector is fetched from the sectorToSendRegisters
    // But it's hard coded to 1 in libdragon's pc64 patch
    send_data(1);
}

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
    // // See FatFs - Generic FAT Filesystem Module, "Application Interface",
	// // http://elm-chan.org/fsw/ff/00index_e.html
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

void test_read_psram(const char* filename) {
    char buf[512];
    sd_card_t *pSD = sd_get_by_num(0);
	FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	if (FR_OK != fr) {
		panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
	}

	FIL fil;

	printf("\n\n---- read /%s -----\n", filename);

	fr = f_open(&fil, filename, FA_OPEN_EXISTING | FA_READ);
	if (FR_OK != fr && FR_EXIST != fr) {
		panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
	}

	FILINFO filinfo;
	fr = f_stat(filename, &filinfo);
	printf("%s [size=%llu]\n", filinfo.fname, filinfo.fsize);

    // TODO read the file header and lookup the eeprom save size

    // printf("Sending eeprom type to mcu1\n");
    // uart_tx_program_putc(COMMAND_START);
    // uart_tx_program_putc(COMMAND_START2);
    // uart_tx_program_putc(COMMAND_SET_EEPROM_TYPE);
    // uart_tx_program_putc(0);
    // uart_tx_program_putc(2);
    // uart_tx_program_putc((uint8_t)(EEPROM_TYPE_4K >> 8));
    // uart_tx_program_putc((uint8_t)(EEPROM_TYPE_4K));

    // // Busy wait for a few cycles then send eeprom data
    // for(int i = 0; i < 10000; i++) { tight_loop_contents(); }

    // load_eeprom_from_sd();

    // for(int i = 0; i < 10000; i++) { tight_loop_contents(); }

    int currentPSRAMChip = START_ROM_LOAD_CHIP_INDEX;
    qspi_enable_spi(4, currentPSRAMChip);
    int numChipsToUse = 2;

	int len = 0;
	int total = 0;
	uint64_t t0 = to_us_since_boot(get_absolute_time());
    
	// do {
        fr = f_read(&fil, buf, sizeof(buf), &len);
        uint32_t addr = 0;//total - ((currentPSRAMChip - START_ROM_LOAD_CHIP_INDEX) * PSRAM_CHIP_CAPACITY_BYTES);
        // program_write_buf(addr, buf, len);
        // Write data to ALL the chips
        for(int i = 1; i <= numChipsToUse; i++) {
            psram_set_cs(i);
            qspi_spi_write_buf(0, buf, len);

            if (i == 1) {
                char psram_buf[256] = {0};
                qspi_spi_read_data(0, psram_buf, 256);
                for (int k = 0; k < 256; k++) {
                    if (k % 8 == 0) { printf("\n%08x: ", k); }
                    printf("%02x ", psram_buf[k]);
                }
                printf("\n\n");
            }
        }
		total += len;

        int newChip = psram_addr_to_chip(total);
        if (newChip != currentPSRAMChip && newChip <= MAX_MEMORY_ARRAY_CHIP_INDEX) {
            printf("Changing memory array chip. Was: %d, now: %d\n", currentPSRAMChip, newChip);
            printf("Total bytes: %d. Bytes remaining = %ld\n", total, (filinfo.fsize - total));
            currentPSRAMChip = newChip;
            psram_set_cs(currentPSRAMChip); // Switch the PSRAM chip
            // break;
        }

	// } while (len > 0);

	fr = f_close(&fil);
	if (FR_OK != fr) {
		printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
	}
	printf("---- read file done -----\n\n\n");

    qspi_enable_qspi(START_ROM_LOAD_CHIP_INDEX, numChipsToUse);

    // reopen the file
    fr = f_open(&fil, filename, FA_OPEN_EXISTING | FA_READ);
	if (FR_OK != fr && FR_EXIST != fr) {
		panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
	}
    // Reset variables
    total = 0;
    len = 0;
    currentPSRAMChip = START_ROM_LOAD_CHIP_INDEX;
    psram_set_cs(currentPSRAMChip); // Switch the PSRAM chip

    int whole_array_error_count = 0;
    int chipErrorCount[9] = {0};
    // do {
        fr = f_read(&fil, buf, sizeof(buf), &len);
        addr = 0;//total - ((currentPSRAMChip - START_ROM_LOAD_CHIP_INDEX) * PSRAM_CHIP_CAPACITY_BYTES);
        
        volatile uint16_t* buf_16 = (volatile uint16_t*)buf;
        
        len = 64;

        for (int c = 1; c <= numChipsToUse; c++) {
            printf("\nChip %d\n", c);
            psram_set_cs(c);
            sleep_ms(10);
            // qspi_init_qspi();
            for (int l = 0; l < 1; l++) {
                for(int i = 0; i < 256; i++) {
                    volatile uint16_t *ptr_16 = (volatile uint16_t *)0x13000000;
                    uint16_t word = ptr_16[i];
                    uint16_t b = buf_16[i];
                    
                    if (l == 0) {
                        if (i % 4 == 0) { printf("\n[%08x]: ", i*2); }
                        printf("%04x ", word);
                    }
                    if(b != word) {
                        whole_array_error_count++;
                        // if (chipErrorCount[c] < 16) {
                        //     printf("\nERROR! [%08x]%04x != %04x\n", addr+(i*2), b, word);
                        // }
                        chipErrorCount[c]++;
                    }
                }
                sleep_ms(10);
            }
            
            qspi_qspi_exit_quad_mode();
        }

		total += len;

        // int newChip = psram_addr_to_chip(total);
        // if (newChip != currentPSRAMChip && newChip <= MAX_MEMORY_ARRAY_CHIP_INDEX) {
        //     // printf("Changing memory array chip. Was: %d, now: %d\n", currentPSRAMChip, newChip);
        //     // printf("Total bytes: %d. Bytes remaining = %ld\n", total, (filinfo.fsize - total));
        //     exitQuadMode();
        //     sleep_ms(10);

        //     currentPSRAMChip = newChip;
        //     psram_set_cs(currentPSRAMChip); // Switch the PSRAM chip
        // }

	// } while (len > 0);
    printf("\n");
    for(int i = 1; i <= numChipsToUse; i++) {
        printf("%d errors on chip %d\n", chipErrorCount[i], i);
    }
    printf("Found %d errors looking at all data\n\n", whole_array_error_count);

    // Now turn off the hardware
    qspi_disable();
    return;






    volatile uint16_t* buf16 = (volatile uint16_t*)buf;
    volatile uint16_t *ptr = (volatile uint16_t *)0x13000000;

    // Dump the buffer
    for(int i = 0; i < sizeof(buf)/2; i++) {
        if (i % 16 == 0) {
            printf("\n");
        }
        printf("[%d]%04x ",i, buf16[i]);
    }

    uint32_t startTestTime = time_us_32();
    printf("TIME: %uus %ums\n", startTestTime, startTestTime/1000);

    // int index1 = 0;
    // volatile uint16_t tempCheck = 0;
    // while(true) {

    //     tempCheck = ptr[index1];

    //     index1++;
    //     if (index1 >= 1024 * 8) {
    //         index1 = 0;
    //     }
    // }

    // int error_count_random_test = 0;
    // srand(time_us_32());
    // for(int i = 0; i < 8 * 1024 * 1024; i++) {
    //     int r = (rand() % 8) + 1;
    //     psram_set_cs(r);

    //     int addr = (rand() % 256);
    //     if(ptr[addr] != buf16[addr]) {
    //         error_count_random_test++;
    //         printf("C%d ", r);
    //     }
    // }
    // printf("\nFound %d errors during random read test.\n", error_count_random_test);

    // Now enable xip and try to read
    int totalErrorsFound = 0;
    for (int o = 1; o <= 8; o++) {
        psram_set_cs(o); // Use the PSRAM chip
        program_flash_enter_cmd_xip(true);
        sleep_ms(1);

        printf("\n\nCheck data from U%u...\n", o);

        // Do a check of the data against the buffer
        for(int run = 0; run < 100; run++) {
            int errors = 0;
            for(int i = 0; i < 256; i++) {
                uint16_t word = ptr[i];
                if(word != buf16[i]) {
                    // Limit the number of errors printed...
                    // if (errors < 16) {
                    //     printf("[%d]%04x != %04x ", i, word, buf16[i]);
                    // }
                    errors++;
                }
            }

            if (errors > 0) {
                printf("\nRun %d: Found %d errors on chip [%d]\n", run, errors, o);
            }
            totalErrorsFound += errors;
        }
        printf("\n\n");
        
        // for (int i = 0; i < 512; i+=2) {
        //     volatile uint16_t word = ptr[i];
        //     volatile uint16_t word2 = ptr[i+1];
        //     if (i < 32) {
        //         printf("[%08x] %04x %04x\n", i, swap8(word), swap8(word2));
        //     }
        // }

        // exit quad mode once finished with read
        qspi_qspi_exit_quad_mode();
        sleep_ms(100);
    }

    printf("\n\%d total errors reading 8 chips.\n", totalErrorsFound);

    // Now turn off the hardware
    current_mcu_enable_demux(false);
    ssi_hw->ssienr = 0;
    qspi_disable();

    // #if DEBUG_MCU2_PRINT == 1
    // uint32_t startTime_us = time_us_32();
    // int o = 0;
    // for (int i = 0; i < 128; i++) {
    //     volatile uint32_t word = testReadBuf[i];
    //     if (i % 16 == 0) { 
    //         printf("Chip %d\n", o + START_ROM_LOAD_CHIP_INDEX);
    //     }
    //     printf("PSRAM-MCU2[%08x]: %08x\n",i * 4 + (o * 8 * 1024 * 1024), word);

    //     if (i % 16 == 0) {
    //         printf("\n128 32bit reads @ 0x13000000 reads took %u us\n", totalReadTime[o]);
    //         o++;
    //     }
    // }
    
    // #endif

    // printf("Rom Loaded, MCU2 qspi: OFF, sending mcu1 rom loaded command\n");

    // // Let MCU1 know that we are finished
    // uart_tx_program_putc(COMMAND_START);
    // uart_tx_program_putc(COMMAND_START2);
    // uart_tx_program_putc(COMMAND_ROM_LOADED);
    // // Zero bytes to read!
    // uart_tx_program_putc(0x00);
    // uart_tx_program_putc(0x00);
}

