/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */

#pragma once

#include "hardware/structs/ioqspi.h"
#include "hardware/structs/pads_qspi.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/ssi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/dma.h"
#include "hardware/regs/pads_qspi.h"
#include "hardware/regs/io_qspi.h"

#include "gpio_helper.h"

#define QSPI_SCLK_PIN  (0)
#define QSPI_SS_PIN    (1)
#define QSPI_SD0_PIN   (2)
#define QSPI_SD1_PIN   (3)
#define QSPI_SD2_PIN   (4)
#define QSPI_SD3_PIN   (5)

// It seems the PAD order is different
#define QSPI_SCLK_PAD  (0)
#define QSPI_SD0_PAD   (1)
#define QSPI_SD1_PAD   (2)
#define QSPI_SD2_PAD   (3)
#define QSPI_SD3_PAD   (4)
#define QSPI_SS_PAD    (5)

extern bool g_isMCU1;

#define QSPI_QUAD_MODE_CLK_DIVIDER 4 // rp2040@266MHz = 133MHz psram clk

#define LOG_BUFFER_SIZE 2048
extern uint32_t log_buffer[LOG_BUFFER_SIZE]; // store addresses
void add_log_to_buffer(uint32_t value);

extern volatile uint32_t update_rom_cache_for_address;
void load_rom_cache(uint32_t startingAt);
void update_rom_cache(uint32_t address);

void qspi_enable(int clk_divider);
void qspi_disable();
void qspi_oeover_normal(bool enable_ss);
void qspi_oeover_disable(); // Advanced function, generally, use qspi_disable if you are turning off the ssi hardware 
void qspi_enable_spi(int clk_divider, int startingChipIndex);
void qspi_enable_qspi(int startingChipIndex, int lastChipIndex);
void qspi_enable_flash(int clk_divider);
void qspi_init_qspi();
void qspi_qspi_exit_quad_mode();
void qspi_qspi_do_cmd(uint8_t cmd);

void qspi_spi_put_get(const uint8_t *tx, uint8_t *rx, size_t count, size_t rx_skip);
void qspi_spi_do_cmd(uint8_t cmd, const uint8_t *tx, uint8_t *rx, size_t count);
void qspi_spi_put_cmd_addr(uint8_t cmd, uint32_t addr);
void qspi_spi_write_buf(uint32_t addr, const uint8_t* data, uint32_t len);
void qspi_spi_read_data(uint32_t addr, uint8_t *rx, size_t count);
void qspi_spi_wait_ready();

// LEGACY FUNCTION, DON'T USE
void program_flash_enter_cmd_xip(bool isPSRAM);
