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
#include "sha256.h"

#include "stdio_async_uart.h"

#include "gpio_helper.h"
#include "utils.h"

#include "qspi_helper.h"
#include "sdcard/internal_sd_card.h"
#include "pio_uart/pio_uart.h"
#include "psram.h"
#include "flash_array/flash_array.h"
#include "flash_array/program_flash_array.h"

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

	printf("0x%08x ", value);
	if (log_tail >= LOG_BUFFER_SIZE) {
		log_tail = 0;
	}
}

volatile uint16_t* word_buff2;
volatile int sentWord2 = 1;
uint16_t t(int dma_chan) {
	if (sentWord2 == 1) {
		dma_channel_start(dma_chan);
		dma_channel_wait_for_finish_blocking(dma_chan);
	}
	
	volatile uint16_t t = word_buff2[sentWord2--];

	if (sentWord2 == -1) {
		sentWord2 = 1;
	}

	return t;
}

volatile uint16_t fetched_word32 = 0;
void testReadRomData() {
	volatile uint16_t *ptr = (volatile uint16_t *)0x13000000;
	volatile uint32_t *ptr32 = (volatile uint32_t *)0x13000000;
	volatile uint16_t startingWord = ptr[0];
	printf("\n\nStarting word: %08x\n", startingWord);

	int clk_step = 10;

	uint32_t cycleCountStart = 0;
	uint32_t totalTime = 0;
	int psram_csToggleTime = 0;
	int total_memoryAccessTime = 0;
	int totalReadTime = 0;
	uint32_t endTimeBuffer[128] = {0};
	uint32_t startTimeBuffer[128] = {0};
	systick_hw->csr = 0x5;
    systick_hw->rvr = 0x00FFFFFF;

	// while(1) {

	for (int o = 0; o < 128; o++) {
		uint32_t modifiedAddress = o;
		uint32_t startTime_us = time_us_32();
		startTimeBuffer[o] = systick_hw->cvr;
		uint16_t word16 = ptr[modifiedAddress];
		endTimeBuffer[o] = systick_hw->cvr;
		totalReadTime += time_us_32() - startTime_us;

		if (o < 16) { // only print the first 16 words
			printf("[%08x]: %04x\n", o * 4, word16);
		}
	}

	// 	fetched_word32 = ptr[0];
	// 	sleep_ms(500);
	// }

	printf("128 16bit reads %d us\n\n\n", totalReadTime);

	// uint32_t worst = 0;
	// uint32_t best = UINT32_MAX;
	// uint64_t totalCycles = 0;
	// for (int i = 0; i < 128; i++) {
	// 	uint32_t cycles = 0;
	// 	if (startTimeBuffer[i] > endTimeBuffer[i]) {
	// 		cycles = startTimeBuffer[i] - endTimeBuffer[i];
	// 	} else {
	// 		cycles = endTimeBuffer[i] - startTimeBuffer[i];
	// 	}
		
	// 	if (cycles > worst) {
	// 		worst = cycles;
	// 	}
	// 	if (cycles < best) {
	// 		best = cycles;
	// 	}
		
	// 	totalCycles += (uint64_t)cycles;
	// 	// printf("%u\n", cycles);
	// }

	// printf("Best: %u, worst: %u, average: %u\n", best, worst, (uint32_t)(totalCycles / 128));
	// printf("Best: %uns, worst: %uns, average: %uns\n", (uint32_t)(best * 3.76), (uint32_t)(worst * 3.76), (uint32_t)((totalCycles / 128) * 3.76));

	uint16_t *rxbuf = malloc(128*2);

// Get a free channel, panic() if there are none
    int chan = dma_claim_unused_channel(true);

    // 8 bit transfers. Both read and write address increment after each
    // transfer (each pointing to a location in src or dst respectively).
    // No DREQ is selected, so the DMA transfers as fast as it can.

    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
	channel_config_set_bswap(&c, true);

	word_buff2 = malloc(4);

    dma_channel_configure(
        chan,          // Channel to be configured
        &c,            // The configuration we just created
        word_buff2,           // The initial write address
        ptr,           // The initial read address
        1, // Number of transfers;
        false           
    );
	

    // We could choose to go and do something else whilst the DMA is doing its
    // thing. In this case the processor has nothing else to do, so we just
    // wait for the DMA to finish.

	uint32_t startTime_us = time_us_32();
	for (int i = 0; i < 128; i++) {
		// dma_channel_start(chan);
    	// dma_channel_wait_for_finish_blocking(chan);
		// printf("%04x %04x\n", rxbuf[i], rxbuf[i+1]);

		// rxbuf[i] = t(chan);

		if (sentWord2 == 1) {
			// dma_channel_start(chan);
			dma_hw->multi_channel_trigger = 1u << chan;
			while(!!(dma_hw->ch[chan].al1_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS)) tight_loop_contents();
			// dma_channel_wait_for_finish_blocking(chan);
		}
		
		rxbuf[i] = word_buff2[sentWord2--];

		if (sentWord2 == -1) {
			sentWord2 = 1;
		}
	}
	uint32_t totalTime_us = time_us_32() - startTime_us;

	for (int i = 0; i < 32; i+=2) {
		printf("%04x %04x\n", rxbuf[i+1], rxbuf[i]);
	}
	printf("DMA 128 16bit reads %uus\n", totalTime_us);

	dma_channel_unclaim(chan);
}

uint32_t last_rom_cache_update_address = 0;
void __no_inline_not_in_flash_func(mcu1_core1_entry)() {	
	// pio_uart_init(PIN_MCU2_DIO, PIN_MCU2_CS); // turn on inter-mcu comms

	bool readingData = false;
	bool startJoybus = false;
	volatile bool isWaitingForRomLoad = false;

	// Some debug and test variables
	volatile bool hasInit = false;
	volatile bool test_load = true;
	volatile uint32_t t = 0;
	volatile uint32_t it = 0;
	volatile uint32_t t2 = 0;
	
	while (1) {
		tight_loop_contents();

		// Tick every second
		if(time_us_32() - t > 1000000) {
			t = time_us_32();
			t2++;
		}

		if (startJoybus) {
			startJoybus = false;
			// enable_joybus();
		}

		// This would typically be used with test load code after a rom has been loaded
		// recompile with test_load off and rom should be ready to boot after a few seconds
		// then power on the n64. Not for the faint of heart.
		if (t2 == 2 && !hasInit && test_load) {
			hasInit = true;
			set_demux_mcu_variables(PIN_DEMUX_A0, PIN_DEMUX_A1, PIN_DEMUX_A2, PIN_DEMUX_IE);
			uint currentChipIndex = START_ROM_LOAD_CHIP_INDEX;
			current_mcu_enable_demux(true);
			psram_set_cs(currentChipIndex);
			program_connect_internal_flash();
			program_flash_exit_xip();

			psram_set_cs(currentChipIndex);
			program_flash_do_cmd(0x35, NULL, NULL, 0);

			psram_set_cs(currentChipIndex + 1);
			program_flash_do_cmd(0x35, NULL, NULL, 0);

			psram_set_cs(currentChipIndex + 2);
			program_flash_do_cmd(0x35, NULL, NULL, 0);

			psram_set_cs(currentChipIndex + 3);
			program_flash_do_cmd(0x35, NULL, NULL, 0);

			// Flush cache
			program_flash_flush_cache();

			program_flash_enter_cmd_xip(true); // psram quad mode

			psram_set_cs(START_ROM_LOAD_CHIP_INDEX); // Set back to start index

			testReadRomData();

			// rom is loaded now
			g_loadRomFromMemoryArray = true; // read from psram
			isWaitingForRomLoad = false;
			sd_is_busy = false;
			readingData = false;
			startJoybus = true;
		}

		// Do a rom load test after x seconds
		// This is useful for loading a rom into psram while the device is
		// connected to usb power for quicker rom loads and code testing.
		// Bit of a hack to get working and probably not for the faint of heart.
		// if(test_load && t2 > 1) {
		// 	test_load = false;
		// 	printf("Disabling mcu1 qspi\n");
		// 	// pc64_set_sd_rom_selection("Donkey Kong 64 (U) [!].z64", 27);
		// 	//pc64_set_sd_rom_selection("GoldenEye 007 (U) [!].z64", 27);
		// 	// pc64_set_sd_rom_selection("007 - The World is Not Enough (U) [!].z64", 42);
		// 	// sd_is_busy = true;
		// 	// romLoading = true;
		// 	// isWaitingForRomLoad = true;
			
		// 	// readingData = true;
		// 	// rx_uart_buffer_reset();

		// 	// Turn off the qspi hardware so mcu2 can use it
		// 	current_mcu_enable_demux(false);
		// 	ssi_hw->ssienr = 0;
		// 	qspi_disable();

		// 	// Something about the above code to turn off qspi
		// 	// causing the pi loop to behave oddly.
		// 	// This will restart the loop.
		// 	g_restart_pi_handler = true;

		// 	// pc64_send_load_new_rom_command();
		// }

		// Handle joy_bus requests
		// if (process_joybus_buf) {
		// 	read_joybus();
		// }

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
				current_mcu_enable_demux(true);
				psram_set_cs(currentChipIndex);
				program_connect_internal_flash();
				program_flash_exit_xip();

				psram_set_cs(currentChipIndex);
				program_flash_do_cmd(0x35, NULL, NULL, 0);

				psram_set_cs(currentChipIndex + 1);
				program_flash_do_cmd(0x35, NULL, NULL, 0);

				psram_set_cs(currentChipIndex + 2);
				program_flash_do_cmd(0x35, NULL, NULL, 0);

				psram_set_cs(currentChipIndex + 3);
				program_flash_do_cmd(0x35, NULL, NULL, 0);

				// Flush cache
				program_flash_flush_cache();

				program_flash_enter_cmd_xip(true); // psram quad mode

				psram_set_cs(START_ROM_LOAD_CHIP_INDEX); // Set back to start index

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

				// pio_uart_stop();
				// enable_joybus();
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
					
					pc64_send_sd_read_command();
				case CORE1_UPDATE_ROM_CACHE:
					printf("Not using cache. Uncomment code to load cache...\n");
					// if (last_rom_cache_update_address != update_rom_cache_for_address) {
					// 	update_rom_cache(update_rom_cache_for_address);
					// 	last_rom_cache_update_address = update_rom_cache_for_address;
					// 	printf("Cache update %08x\n", update_rom_cache_for_address);
					// }
					break;
				case CORE1_LOAD_NEW_ROM_CMD:
					sd_is_busy = true;
					romLoading = true;
					isWaitingForRomLoad = true;
					
					readingData = true;
					rx_uart_buffer_reset();

					// Turn off the qspi hardware so mcu2 can use it
					current_mcu_enable_demux(false);
    				ssi_hw->ssienr = 0;
    				qspi_disable();

					// Something about the above code to turn off qspi
					// causing the pi loop to behave oddly.
					// This will restart the loop.
					g_restart_pi_handler = true;

					pc64_send_load_new_rom_command();

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
	// const int freq_khz = 250000;
	const int freq_khz = 266000;
	// NOTE: For speeds above 266MHz voltage must be increased.
	// const int freq_khz = 300000;

	// IMPORTANT: For the serial comms between mcus to work properly 
	// both mcus must be run at the same clk speed or have the pio divder set accordingly

	// Note that this might call set_sys_clock_pll,
	// which might set clk_peri to 48 MHz
	bool clockWasSet = set_sys_clock_khz(freq_khz, false);

	gpio_configure(mcu1_gpio_config, ARRAY_SIZE(mcu1_gpio_config));

	// Enable STDIO, typically disabled on mcu1 as the uart pin is being used
	// for serial comms to mcu2.
	// stdio_async_uart_init_full(DEBUG_UART, DEBUG_UART_BAUD_RATE, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
	stdio_uart_init_full(DEBUG_UART, DEBUG_UART_BAUD_RATE, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);

	printf("\n\nMCU1: Was%s able to set clock to %d MHz\n", clockWasSet ? "" : " not", freq_khz/1000);

	// IF READING FROM FROM FLASH... (works for compressed roms)
	// Enabled to boot menu rom
	qspi_oeover_normal(true);
	ssi_hw->ssienr = 0;
	ssi_hw->baudr = 8; // change baud
	ssi_hw->ssienr = 1;

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
