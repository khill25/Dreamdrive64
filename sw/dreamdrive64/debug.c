/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include "debug.h"

void mcu2_debug_load_rom() {
    // qspi_disable();
	// ssi_hw->ssienr = 0;
	// qspi_oeover_disable();

	// test_read_psram("Resident Evil 2 (USA) (Rev 1).z64");
	// test_read_psram("Pokemon Stadium 2 (USA).z64");
	// test_read_psram("GoldenEye 007 (U) [!].z64");

	// load_new_rom("GoldenEye 007 (U) [!].z64");
	// load_new_rom("Donkey Kong 64 (U) [!].z64");
	// load_new_rom("Legend of Zelda, The - Majora's Mask (U) [!].z64");
	// load_new_rom("Perfect Dark (U) (V1.1) [!].z64");
	// load_new_rom("Resident Evil 2 (USA) (Rev 1).z64");
	// load_new_rom("Pokemon Stadium 2 (USA).z64");
	// load_new_rom("1080[en,jp].z64");
	// load_new_rom("Legend of Zelda, The - Ocarina of Time (U) (V1.2) [!].z64");

	// mcu2_setup_verify_rom_data(); // opens the file into some global variables
}

#if IS_DOING_READ_TEST == 1

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

#endif