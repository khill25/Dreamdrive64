/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill <kaili.hill25@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pc64_regs.h"

#define SD_CARD_SECTOR_SIZE 512 // 512 bytes

extern int PC64_MCU_ID;
extern __volatile bool sendDataReady;
extern __volatile uint32_t sectorToSend;

// Communucation buffer (Also used for sd access from roms)
extern __volatile uint16_t pc64_uart_tx_buf[PC64_BASE_ADDRESS_LENGTH];
extern __volatile bool sd_is_busy;
extern uint8_t diskReadBuffer[SD_CARD_SECTOR_SIZE];

// set the sector to start reading from
void pc64_set_sd_read_sector(uint32_t sector);

void pc64_set_sd_read_sector_part(int index, uint16_t value);

// set the number of sectors to read
void pc64_set_sd_read_sector_count(uint32_t count);

// Set selected rom title, max 256 characters
void pc64_set_sd_rom_selection(char* title);

void pc64_send_sd_read_command(void);

void process_mcu2_cmd_buffer(unsigned char* mcu2_cmd_buffer, int len);
// void send_sd_card_data();
void send_data(uint32_t sector, uint32_t sectorCount);

// SD Card functions
void mount_sd(void);
