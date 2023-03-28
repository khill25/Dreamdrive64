/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 * Copyright (c) 2022 Kaili Hill
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/ssi.h"
#include "hardware/structs/systick.h"
#include "pico/time.h"
#include "hardware/vreg.h"

#include "pins_mcu1.h"
#include "n64_pi_task.h"
#include "reset_reason.h"
// #include "sha256.h"

#include "stdio_async_uart.h"

#include "gpio_helper.h"
#include "utils.h"

#include "qspi_helper.h"
#include "sdcard/internal_sd_card.h"
#include "pio_uart/pio_uart.h"
#include "psram.h"

#include "rom_vars.h"

#include "joybus/joybus.h"

static const gpio_config_t mcu1_gpio_config[] = {
	// PIO0 pins
	{PIN_N64_AD0, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD1, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD2, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD3, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD4, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD5, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD6, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD7, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD8, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD9, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD10, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD11, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD12, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD13, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD14, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_AD15, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_ALEL, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_ALEH, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_WRITE, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},
	{PIN_N64_READ, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO0},

	// Remaining N64 pins are treated as normal GPIOs
	{PIN_N64_COLD_RESET, GPIO_IN, false, false, true, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},	// Pulled down
	//{PIN_N64_SI_DAT, GPIO_IN, false, true, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},	// Pulled up, open drain
	{PIN_N64_INT1, GPIO_IN, false, true, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},	// Pulled up, open drain

	// Demux should be configured as inputs without pulls until we lock the bus
	{PIN_DEMUX_A0, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_DEMUX_A1, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_DEMUX_A2, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_DEMUX_IE, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},

	// Initial config for serial comm pin, doesn't really matter since stdio init or pio_uart init will
	// init the pin for the correct config.
	{PIN_MCU2_CS, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO1},
	{PIN_MCU2_DIO, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO1},
};

volatile bool g_restart_pi_handler = false;

// Basic ring buffer. Useful for debug info
// Data can be sent to mcu2 instead of stdio by using
// `uart_tx_program_putc` method.
uint32_t log_buffer[LOG_BUFFER_SIZE]; // store addresses
volatile int log_head = 0;
volatile int log_tail = 0;

volatile uint32_t lastLoggedValue = 0;
volatile int compactedSequensialValues = 0;
void add_log_to_buffer(uint32_t value) {
	log_buffer[log_head++] = value;
	if (log_head >= LOG_BUFFER_SIZE) {
		log_head = 0;
	}
}

static uint32_t last_log_value = 0;
void process_log_buffer() {
	if (log_tail == log_head) {
		// noting to print
		return;
	}

	uint32_t value = log_buffer[log_tail++];

	// if (value > 1000) {
	// 	printf("0x%08x ", value);
	// } else {
	// 	printf("%u ", value);
	// }
	// printf("0x%08x ", value);
	while(!uart_tx_program_is_writable()) { tight_loop_contents(); }
	uart_tx_program_putc(0xAA);
	uart_tx_program_putc(value >> 24);
	uart_tx_program_putc(value >> 16);
	uart_tx_program_putc(value >> 8);
	uart_tx_program_putc(value);
	uart_tx_program_putc(0xBB);

	if (log_tail >= LOG_BUFFER_SIZE) {
		log_tail = 0;
	}

	// if (log_tail % 16 == 0) {
	// 	printf("\n");
	// }
}

static int dma_bi = 0;
static uint16_t* dmaBuffer;
uint16_t rom_read_test(int dma_chan) {
	uint16_t next_word = 0;
	if (dma_bi == 0) {
		dma_channel_wait_for_finish_blocking(dma_chan);
		next_word = (uint16_t)dmaBuffer[1];
		// dma_channel_set_write_addr(dma_chan, &dmaBuffer[2], true);
		dma_channel_start(dma_chan);
		dma_bi++;

	} else if (dma_bi == 1) {
		next_word = (uint16_t)dmaBuffer[0];
		dma_bi++;

	} else if (dma_bi == 2) {
		dma_channel_wait_for_finish_blocking(dma_chan);
		next_word = (uint16_t)dmaBuffer[3];
		dma_channel_set_write_addr(dma_chan, &dmaBuffer[0], true); // start load of next word but at position 0, so we wrap our buffer around
		dma_bi++;

	} else if (dma_bi == 3) {
		next_word = (uint16_t)dmaBuffer[2];
		dma_bi = 0;
	}

	return next_word;
}

uint32_t last_rom_cache_update_address = 0;
void __no_inline_not_in_flash_func(mcu1_core1_entry)() {	
	pio_uart_init(PIN_MCU2_DIO, PIN_MCU2_CS); // turn on inter-mcu comms	
	// pio_uart_stop(false, true); // disable rx?

	bool readingData = false;
	bool startJoybus = false;
	volatile bool isWaitingForRomLoad = false;

	// Some debug and test variables
	volatile bool hasInit = false;
	volatile bool test_load = false;
	volatile uint32_t t = 0;
	volatile uint32_t it = 0;
	volatile uint32_t t2 = 0;
	
	while (1) {
		tight_loop_contents();

		// Tick every second
		if(time_us_32() - t > 1000000) {
			t = time_us_32();
			t2++;
			
			// if (t2 == 4) {
			// 	mcu1_process_rx_buffer();
			// }
		}

		// process_log_buffer();

		if (startJoybus) {
			startJoybus = false;
			// Joybus currently runs in a while loop. 
			// Running the joybus means that other code here
			// will not run once joybus is started.
			enable_joybus(); 
		}

		// This would typically be used with test load code after a rom has been loaded
		// recompile with test_load off and rom should be ready to boot after a few seconds
		// then power on the n64. Not for the faint of heart.
		if (t2 == 1 && !hasInit && test_load) {
			hasInit = true;
			set_demux_mcu_variables(PIN_DEMUX_A0, PIN_DEMUX_A1, PIN_DEMUX_A2, PIN_DEMUX_IE);
			uint currentChipIndex = START_ROM_LOAD_CHIP_INDEX;
			qspi_enable_qspi(currentChipIndex, MAX_MEMORY_ARRAY_CHIP_INDEX);

			// for(int i = 1; i <= 8; i++) {
			// 	qspi_qspi_do_cmd(0x0C);
			// }

			// testReadRomData();
			// verify_rom_data();

			volatile uint16_t *ptr16 = (volatile uint16_t *)0x13000000;
			volatile uint16_t word = ptr16[0];
			printf("%04x\n", word);

			// Exit quad mode
			// for(int i = 1; i <= 8; i++) {
			// 	psram_set_cs(i);
			// 	qspi_qspi_exit_quad_mode();
			// 	sleep_ms(10);
			// }

			// rom is loaded now
			g_loadRomFromMemoryArray = true; // read from psram
			isWaitingForRomLoad = false;
			sd_is_busy = false;
			readingData = false;

			// disable uart rx
			// pio_uart_stop(false, true);

			// Start joybus
			// startJoybus = true;
		}

		if (readingData) {
			// Process anything that might be on the uart buffer
			mcu1_process_rx_buffer();

			if (sendDataReady && !isWaitingForRomLoad) {
				// Now that the data is written to the array, go ahead and release the lock
				sd_is_busy = false;
				readingData = false;
			} else if (sendDataReady && isWaitingForRomLoad) {
				set_demux_mcu_variables(PIN_DEMUX_A0, PIN_DEMUX_A1, PIN_DEMUX_A2, PIN_DEMUX_IE);
				uint currentChipIndex = START_ROM_LOAD_CHIP_INDEX;
				qspi_enable_qspi(currentChipIndex, MAX_MEMORY_ARRAY_CHIP_INDEX);

				// After the rom has been loaded we can choose to validate the data
				// by reading from mcu1 and sending it to mcu2 to verify.
				// This provides a sanity check that Mcu1 can correctly read
				// data from the psram array.
				// verify_rom_data();

				// rom is loaded now
				g_loadRomFromMemoryArray = true; // read from psram
				isWaitingForRomLoad = false;
				sd_is_busy = false;
				readingData = false;

				// disable uart rx
				pio_uart_stop(false, true);
				// start joybus
				startJoybus = true;

				// Sanity chirp to mcu2 just to know that this completed
				uart_tx_program_putc(0xAB);
			}
		}

		if (multicore_fifo_rvalid()) {
			int32_t cmd = multicore_fifo_pop_blocking();
			switch (cmd) {
				case CORE1_SEND_SD_READ_CMD:
					// Block cart while waiting for data
					sd_is_busy = true;

					// Finally start processing the uart buffer
					readingData = true;
					rx_uart_buffer_reset();
					
					ddr64_send_sd_read_command();
					break;

				case CORE1_LOAD_NEW_ROM_CMD:
					sd_is_busy = true;
					romLoading = true;
					isWaitingForRomLoad = true;
					
					readingData = true;
					rx_uart_buffer_reset();

					// Turn off the qspi hardware so mcu2 can use it
					qspi_disable();

					// Something about the above code to turn off qspi
					// causing the pi loop to behave oddly.
					// This will restart the loop.
					g_restart_pi_handler = true;

					ddr64_send_load_new_rom_command();

					break;

				default:
					break;
			}
		}
	}
}

const uint32_t timeout_us = 10000 * 1000; // 10 seconds
int test_line_read(int gpio, char* name) {
	uint32_t start_time = time_us_32();
	// uint32_t current_time = start_time;
	int value = gpio_get(gpio);
	while(value == false) {
		tight_loop_contents();
		if ((time_us_32()-start_time) >= timeout_us) {
			printf("Timeout waiting for %s [gpio %d] to return value\n", name, gpio);
			break;
		}
		// current_time += time_us_32() - start_time;

		value = gpio_get(gpio);
	}

	if (value) {
		printf("%s [GPIO %d] good!\n", name, gpio);
	}

	return value;
}

void boardTest() {

	// Init the pins and set them all to input

	// init the address/data pins
	for(int i = 0; i < 16; i++) {
		gpio_init(i);
		gpio_set_dir(i, false);
	}

	gpio_init(PIN_N64_ALEH);
    gpio_set_dir(PIN_N64_ALEH, false);
    
    gpio_init(PIN_N64_ALEL);
    gpio_set_dir(PIN_N64_ALEL, false);
    
    gpio_init(PIN_N64_READ);
    gpio_set_dir(PIN_N64_READ, false);
    
    gpio_init(PIN_N64_COLD_RESET);
    gpio_set_dir(PIN_N64_COLD_RESET, false);

	// Wait until the cold reset is true
	while(gpio_get(PIN_N64_COLD_RESET) == false) {
		tight_loop_contents();
	}

	uint32_t start_time = time_us_32();
	uint32_t current_time = start_time;
	// Test data line get values
	printf("\n\nTesting data lines\n");
	for(int i = 0; i < 16; i++) {
		int value = gpio_get(i);

		start_time = time_us_32();
		// current_time = start_time;
		while(!value) {
			if ((time_us_32()-start_time) >= timeout_us) {
				printf("Timeout waiting for gpio %d to return value\n", i);
				break;
			}

			value = gpio_get(i);
		}

		if (value) {
			printf("GPIO/AD [%d] good!\n", i);
		}
	}

	printf("\n\nTesting control lines\n");
	int aleh_good = test_line_read(PIN_N64_ALEH, "ALEH");
	int alel_good = test_line_read(PIN_N64_ALEL, "ALEL");
	int read_good = test_line_read(PIN_N64_READ, "READ");

	// Now test writing to each of the data lines
	for(int i = 0; i < 16; i++) {
		gpio_set_dir(i, true);
		gpio_put(i, true);
	}

	sleep_ms(10);
	// Wait for the read line to go high and signal the end of the test
	test_line_read(PIN_N64_READ, "READ");

	printf("board_test finished!\n");

}

void __no_inline_not_in_flash_func(mcu1_main)(void)
{
	int count = 0;
	// const int freq_khz = 133000;
	// const int freq_khz = 166000;
	// const int freq_khz = 200000;
	// const int freq_khz = 210000;
	// const int freq_khz = 250000;
	// const int freq_khz = 266000;
	// NOTE: For speeds above 266MHz voltage must be increased.
	// const int freq_khz = 300000;
	// const int freq_khz = 332000;
	// const int freq_khz = 360000;
	// const int freq_khz = 384000;
	// const int freq_khz = 400000;

	// IMPORTANT: For the serial comms between mcus to work properly 
	// both mcus must be run at the same clk speed or have the pio divder set accordingly

	// Note that this might call set_sys_clock_pll,
	// which might set clk_peri to 48 MHz
	// vreg_set_voltage(VREG_VOLTAGE_1_25);
	// bool clockWasSet = set_sys_clock_khz(freq_khz, false);

	gpio_configure(mcu1_gpio_config, ARRAY_SIZE(mcu1_gpio_config));

	// Enable STDIO, typically disabled on mcu1 as the uart pin is being used
	// for serial comms to mcu2.
	// stdio_async_uart_init_full(DEBUG_UART, DEBUG_UART_BAUD_RATE, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
	// stdio_uart_init_full(DEBUG_UART, DEBUG_UART_BAUD_RATE, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);

	// printf("\n\nMCU1: Was%s able to set clock to %d MHz\n", clockWasSet ? "" : " not", freq_khz/1000);

	// IF READING FROM FROM FLASH... (works for compressed roms)
	// Enabled to boot menu rom
	set_demux_mcu_variables(PIN_DEMUX_A0, PIN_DEMUX_A1, PIN_DEMUX_A2, PIN_DEMUX_IE);
	qspi_enable_flash(4);

	// Set up ROM mapping table
	if (memcmp(picocart_header, "picocartcompress", 16) == 0) {
		// Copy rom compressed map from flash into RAM
		// uart_tx_program_puts("Found a compressed ROM\n");
		printf("Found a compressed ROM\n");
		memcpy(rom_mapping, flash_rom_mapping, MAPPING_TABLE_LEN * sizeof(uint16_t));
	} else {
		for (int i = 0; i < MAPPING_TABLE_LEN; i++) {
			rom_mapping[i] = i;
		}
	}

	multicore_launch_core1(mcu1_core1_entry);

	printf("launching n64_pi_run...\n");

	n64_pi_run();

	while (true) {
		if(g_restart_pi_handler) {
			n64_pi_run();
		}
	}
}
