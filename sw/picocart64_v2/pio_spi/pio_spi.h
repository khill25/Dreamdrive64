/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill
 */

#ifndef _PIO_SPI_H
#define _PIO_SPI_H

#include "hardware/pio.h"
#include "pio_spi.pio.h"

typedef struct pio_spi_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_spi_inst_t;

enum  {
  SEND_SD_READ_CMD, // then needs the sector to read and number of sectors to read
  SD_DATA_READ_FINISHED,
};

void init_pio_spi(bool isMaster, int clkPin, int rxPin, int txPin);

void pio_spi_write8_blocking(const pio_spi_inst_t *spi, const uint8_t *src, size_t len);
void pio_spi_read8_blocking(const pio_spi_inst_t *spi, uint8_t *dst, size_t len);

// All of these functions are blocking
void pio_spi_putc(const uint8_t src);
void pio_spi_write8(const uint8_t *src, size_t len);
void pio_spi_read8(uint8_t *dst, size_t len);
void pio_spi_read16(uint16_t *dst, size_t len);
void pio_spi_write8_read8(uint8_t *src, uint8_t *dst, size_t len);

#endif