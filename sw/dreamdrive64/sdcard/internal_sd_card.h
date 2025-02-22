/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill <kaili.hill25@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "ddr64_regs.h"
#include "pio_uart/pio_uart.h"

#define ERASE_AND_WRITE_TO_FLASH_ARRAY 0
#define LOAD_TO_PSRAM_ARRAY 2 // 1 if use psram, 0 to use flash, 2 = do nothing?
#define SD_CARD_SECTOR_SIZE 512 // 512 bytes

extern int DDR64_MCU_ID;
extern volatile bool sendDataReady;
extern volatile bool startRomLoad;
extern volatile bool romLoading;
extern volatile bool start_saveEeepromData;
extern volatile bool start_loadEeepromData;
extern volatile bool start_saveSramData;
extern volatile bool start_loadSramData;
extern volatile bool is_verifying_rom_data_from_mcu1;
extern volatile uint32_t verifyDataTime;
extern volatile int selected_rom_save_type;
extern volatile int selected_rom_cic;
extern volatile int selected_rom_cic_region;

extern volatile bool did_write_SRAM;

// UART TX buffer
extern volatile uint16_t ddr64_uart_tx_buf[DDR64_BASE_ADDRESS_LENGTH];

// set the sector to start reading from
void ddr64_set_sd_read_sector(uint64_t sector);

void ddr64_set_sd_read_sector_part(int index, uint32_t value);

// set the number of sectors to read
void ddr64_set_sd_read_sector_count(int index, uint32_t count);

// Set the length of selected rom title
void ddr64_set_sd_rom_selection_length_register(uint32_t value, int index);

// Set selected rom title, max 256 characters
void ddr64_set_sd_rom_selection(char* titleBuffer, uint32_t len);

void ddr64_set_rom_meta_data(uint32_t value, int index);

void ddr64_send_sd_read_command(void);

// MCU1 will call this method to send the sram contents to mcu2
void send_SRAM_data();

extern volatile bool sd_is_busy;
//bool is_sd_busy();

// UART RX methods, unique per MCU
void mcu1_process_rx_buffer();
void mcu2_process_rx_buffer();

// Variables are extracted via mcu2_process_rx_buffer
// Then data is read from the SD Card and sent over uart
void send_sd_card_data();

// SD Card functions
void mount_sd(void);

/* sd/rom/psram stuff */
// loads the rom file specified in sd_selected_rom_title, that is set with the load rom command from mcu1
void load_selected_rom();
void load_rom(const char *filename);

void ddr64_send_load_new_rom_command();
void load_new_rom(char* filename);

void start_eeprom_sd_save();
void start_sram_sd_save();

void test_read_psram(const char* filename);

void verify_rom_data(); // Used by MCU1 to send data back to MCU2 for verification
void mcu2_setup_verify_rom_data();
void mcu2_verify_sent_rom_data();