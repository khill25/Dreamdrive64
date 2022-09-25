/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */

#include <stdio.h>
#include "pico/stdlib.h"

#include "pins_mcu2.h"
#include "git_info.h"
#include "stdio_async_uart.h"

#include "psram_array.h"

#define UART0_BAUD_RATE  (115200)

RamArrayConfig v2StandardDefaultConfig = {
    .byteCapacity = 8388608,
    .numChipsInArray = 8,
    .clockSpeed = 84 * 1000000, // 84 MHZ. Just use the lower bound as default because we may cross a 1k page boundry and the chip internally limits the speed to 84MHZ in this case.
    .waitCycles = 6,
    .CMD_READ = CMD_FAST_READ_QUAD,
    .CMD_WRITE = CMD_FAST_WRITE_QUAD,
    // Pin configuration will vary based on which MCU is referencing
    .clk_pin = QSPI_SCLK_PIN,
    .cs_mux_pin0 = PIN_DEMUX_A0,
    .cs_mux_pin1 = PIN_DEMUX_A1,
    .cs_mux_pin2 = PIN_DEMUX_A2,
    .ie_pin = PIN_DEMUX_IE,
    .io_pin_0 = QSPI_SD0_PIN,
    .io_pin_1 = QSPI_SD1_PIN,
    .io_pin_2 = QSPI_SD2_PIN,
    .io_pin_3 = QSPI_SD3_PIN,
};


void runRamArrayTests() {
	sleep_ms(1000); // delay before running tests just to be sure we are finished writing to MCU1

	printf("Starting Ram Array tests...\n");
	RamArrayConfig config = v2StandardDefaultConfig;
	init_ram_array(config);

	sleep_ms(50);

	uint32_t address = 0;
	uint32_t numBytes = 100;
	uint32_t writeBuffer[100];
	uint32_t readBuffer[100];

	for (int i = 0; i < numBytes; i++) {
		writeBuffer[i] = i; // set the data to be the index
		readBuffer[i] = 0; // null out the read buffer
	}

	// Write to array
	psramArray_write(config, address, numBytes, writeBuffer);

	// Read the data back
	psramArray_read(config, address, numBytes, readBuffer);

	// Check the data
	for (int i = 0; i < numBytes; i++) {
		if (readBuffer[i] != writeBuffer[i]) {
			printf("%d expected: %d, got %d\n", i, writeBuffer[i], readBuffer[i]);
		}
	}

	printf("Ram Array tests finshed.\n");
}

void mcu2_main(void)
{
	int count = 0;

	// Init async UART on pin 0/1
	//stdio_async_uart_init_full(uart0, UART0_BAUD_RATE, PIN_UART0_TX, PIN_UART0_RX);
	stdio_usb_init();

	printf("PicoCart64 Boot (git rev %08x)\r\n", GIT_REV);

	// TODO: Remove later
	sleep_ms(1000); // Sleeps crash when using the debugger :(

	// Boot MCU1
	gpio_set_dir(PIN_MCU1_RUN, GPIO_OUT);
	gpio_put(PIN_MCU1_RUN, 1);	// Set GPIO19 / MCU2.RUN to HIGH

	runRamArrayTests();

	while (true) {
		count++;
		printf("Hello, world! I am MCU 2 -- %d\n", count);
		sleep_ms(1000); // Sleeps crash when using the debugger :(
	}
}
