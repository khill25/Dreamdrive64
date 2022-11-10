/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill
 */

#include "pio_spi.h"

// Just 8 bit functions provided here. The PIO program supports any frame size
// 1...32, but the software to do the necessary FIFO shuffling is left as an
// exercise for the reader :)
//
// Likewise we only provide MSB-first here. To do LSB-first, you need to
// - Do shifts when reading from the FIFO, for general case n != 8, 16, 32
// - Do a narrow read at a one halfword or 3 byte offset for n == 16, 8
// in order to get the read data correctly justified. 

pio_spi_inst_t pio_spi = {
    .pio = pio1,
    .sm = 0,
    .cs_pin = -1
};

void init_pio_spi_master(int clkPin, int rxPin, int txPin) {
    uint offset = pio_add_program(pio_spi.pio, &spi_master_program);
    pio_spi_master_init(pio_spi.pio, pio_spi.sm, offset,
                        32,       // 8 bits per SPI frame
                        32,  // 1 MHz @ 125 clk_sys
                        false,   // CPOL = 0
                        clkPin,
                        txPin,
                        rxPin
    );
}

void init_pio_spi_slave(int rxPin, int txPin) {
    uint offset = pio_add_program(pio_spi.pio, &spi_slave_program);
    pio_spi_slave_init(pio_spi.pio, pio_spi.sm, offset,
                        32,       // 8 bits per SPI frame
                        2,  // 1 MHz @ 125 clk_sys
                        false,   // CPOL = 0
                        txPin,
                        rxPin
    );
}

void init_pio_spi(bool isMaster, int clkPin, int rxPin, int txPin) {
    if (isMaster) {
        init_pio_spi_master(clkPin, rxPin, txPin);
    } else {
        init_pio_spi_slave(rxPin, txPin);
    }
}

void __time_critical_func(pio_spi_write8_blocking)(const pio_spi_inst_t *spi, const uint8_t *src, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            (void) *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_read8_blocking)(const pio_spi_inst_t *spi, uint8_t *dst, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = 0;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_write8_read8_blocking)(const pio_spi_inst_t *spi, uint8_t *src, uint8_t *dst,
                                                         size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_read16_blocking)(const pio_spi_inst_t *spi, uint16_t *dst, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    uint8_t lastValue = 0;
    int lastValueIndex = 0;
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = 0;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            if (lastValueIndex % 2 == 1) {
                uint16_t value = lastValue << 8 | *rxfifo;
                *dst++ = value;
                --rx_remain;
            } else {
                lastValue = *rxfifo;
            }
            
            lastValueIndex++;
        }
    }
}

void __time_critical_func(pio_spi_read16)(uint16_t *dst, size_t len) {
    pio_spi_read16_blocking(&pio_spi, dst, len);
}

void pio_spi_putc(const uint8_t src) {
    pio_spi_write8_blocking(&pio_spi, &src, 1);
}

void __time_critical_func(pio_spi_write8)(const uint8_t *src, size_t len) {
    pio_spi_write8_blocking(&pio_spi, src, len);
}

void __time_critical_func(pio_spi_read8)(uint8_t *dst, size_t len) {
    pio_spi_read8_blocking(&pio_spi, dst, len);
}

void __time_critical_func(pio_spi_write8_read8)(uint8_t *src, uint8_t *dst, size_t len) {
    pio_spi_write8_read8_blocking(&pio_spi, src, dst, len);
}

