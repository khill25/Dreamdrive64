/**
 * SPDX-License-Identifier: MIT License
 *
 * Copyright (c) 2022 Kaili Hill <kaili.hill25@gmail.com>
 */

#pragma once

#define SD_CARD_SECTOR_SIZE 512 // 512 bytes

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;

// set the sector to start reading from
void pc64_set_sd_read_sector(uint32_t sector);

// set the number of sectors to read
void pc64_set_sd_read_sector_count(uint32_t count);

// Set selected rom title, max 256 characters
void pc64_set_sd_rom_selection(char* title);

// Data is fetched over uart and read data into scratch memory buffer.
// buffer is 4k
void pc64_read_sd(uint32_t sector, uint32_t count);