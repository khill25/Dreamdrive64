/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann <konrad.beckmann@gmail.com>
 * Copyright (c) 2022 Christopher Bonhage <me@christopherbonhage.com>
 *
 * Based on https://github.com/meeq/SaveTest-N64
 */

#include <string.h>
#include <stdint.h>
#include <libdragon.h>

#include "git_info.h"
#include "shell.h"

// picocart64_shared
#include "pc64_regs.h"
#include "pc64_rand.h"
#include "n64_defs.h"
#include "pc64_utils.h"

int main(void)
{
	uint32_t *facit_buf32 = (uint32_t *) facit_buf;
	uint32_t *read_buf32 = (uint32_t *) read_buf;
	uint16_t *read_buf16 = (uint16_t *) read_buf;

	configure_sram();

	display_init(RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);
	console_init();
	// debug_init_isviewer();

	printf("PicoCart64 Test ROM (git rev %08X)\n\n", GIT_REV);

	///////////////////////////////////////////////////////////////////////////

	// Verify PicoCart64 Magic
	data_cache_hit_writeback_invalidate(read_buf, sizeof(read_buf));
	pi_read_raw(read_buf, PC64_CIBASE_ADDRESS_START, PC64_REGISTER_MAGIC, sizeof(uint32_t));
	if (read_buf32[0] == PC64_MAGIC) {
		printf("[ OK ] MAGIC = 0x%08lX.\n", read_buf32[0]);
	} else {
		printf("[FAIL] MAGIC = 0x%08lX.\n", read_buf32[0]);
	}

	///////////////////////////////////////////////////////////////////////////

	// Print Hello from the N64
	strcpy(write_buf, "Hello from the N64!\r\n");
	pc64_uart_write((const uint8_t *)write_buf, strlen(write_buf));
	printf("[ -- ] Wrote hello over UART.\n");

	///////////////////////////////////////////////////////////////////////////

	// Initialize a random sequence
	pc64_rand_seed(0);
	data_cache_hit_writeback_invalidate(facit_buf, sizeof(facit_buf));
	for (int i = 0; i < sizeof(facit_buf) / sizeof(uint32_t); i++) {
		facit_buf32[i] = pc64_rand32();
	}

	// Read back 1Mbit of SRAM
	data_cache_hit_writeback_invalidate(read_buf, sizeof(read_buf));
	pi_read_raw(read_buf, CART_DOM2_ADDR2_START, 0, sizeof(read_buf));

	// Compare SRAM with the facit
	if (memcmp(facit_buf, read_buf, sizeof(read_buf)) != 0) {
		printf("[FAIL] SRAM was not backed up properly.\n");

		for (int i = 0; i < sizeof(facit_buf) / sizeof(uint32_t); i++) {
			if (facit_buf32[i] != read_buf[i]) {
				printf("       Error @%d: facit %08lX != sram %08lX\n", i, facit_buf32[i], read_buf32[i]);
				break;
			}
		}
	} else {
		printf("[ OK ] SRAM was backed up properly.\n");
	}

	///////////////////////////////////////////////////////////////////////////

	// Write 1Mbit of SRAM
	data_cache_hit_writeback_invalidate(facit_buf, sizeof(facit_buf));
	pi_write_raw(facit_buf, CART_DOM2_ADDR2_START, 0, sizeof(facit_buf));

	printf("[ -- ] Wrote test pattern to SRAM.\n");

	// Read it back and verify
	data_cache_hit_writeback_invalidate(read_buf, sizeof(read_buf));
	pi_read_raw(read_buf, CART_DOM2_ADDR2_START, 0, sizeof(read_buf));

	// Compare SRAM with the facit
	if (memcmp(facit_buf, read_buf, sizeof(read_buf)) != 0) {
		printf("[FAIL] SRAM (volatile) did not verify correctly.\n");

		for (int i = 0; i < sizeof(facit_buf) / sizeof(uint32_t); i++) {
			if (facit_buf32[i] != read_buf[i]) {
				printf("       Error @%d: facit %08lX != sram %08lX\n", i, facit_buf32[i], read_buf32[i]);
				break;
			}
		}
	} else {
		printf("[ OK ] (volatile) SRAM verified correctly.\n");
	}

	///////////////////////////////////////////////////////////////////////////

	// Stress test: Read pseudo-random numbers from PicoCart64 RAND address space
	// Reset random seed
	pi_write_u32(0, PC64_CIBASE_ADDRESS_START, PC64_REGISTER_RAND_SEED);
	pc64_rand_seed(0);

	// Compare buffer with RNG
	// printf("[ -- ] RNG Test: ");
	// bool rng_ok = true;
	// for (int j = 0; j < 64 && rng_ok; j++) {
	// 	// Read back 1Mbit of RAND values
	// 	data_cache_hit_writeback_invalidate(read_buf, sizeof(read_buf));
	// 	pi_read_raw(read_buf, PC64_RAND_ADDRESS_START, 0, sizeof(read_buf));
	// 	printf(".");
	// 	fflush(stdout);

	// 	for (int i = 0; i < sizeof(read_buf) / sizeof(uint16_t); i++) {
	// 		uint16_t value = pc64_rand16();

	// 		if (value != read_buf16[i]) {
	// 			printf("\n       Error @%d: ours %04X != theirs %04X", i, value, read_buf16[i]);
	// 			rng_ok = false;
	// 			break;
	// 		}
	// 	}
	// }

	// if (rng_ok) {
	// 	printf("\n[ OK ] Random stress test verified correctly.\n");
	// } else {
	// 	printf("\n[FAIL] Random stress test failed.\n");
	// }

	///////////////////////////////////////////////////////////////////////////

	// Read 1Mbit of 64DD IPL ROM
	data_cache_hit_writeback_invalidate(read_buf, sizeof(read_buf));
	pi_read_raw(read_buf, CART_DOM1_ADDR1_START, 0, sizeof(read_buf));
	printf("[ -- ] Read from the N64DD address space.\n");

	// Verify the PicoCart64 Magic *again*
	// This is done to ensure that the PicoCart64 is still alive and well,
	// and hasn't crashed or locked itself up.
	data_cache_hit_writeback_invalidate(read_buf, sizeof(read_buf));
	pi_read_raw(read_buf, PC64_CIBASE_ADDRESS_START, PC64_REGISTER_MAGIC, sizeof(uint32_t));
	if (read_buf32[0] == PC64_MAGIC) {
		printf("[ OK ] (second time) MAGIC = 0x%08lX.\n", read_buf32[0]);
	} else {
		printf("[FAIL] (second time) MAGIC = 0x%08lX.\n", read_buf32[0]);
		printf("       PicoCart64 might stall now and require a power cycle.\n");
	}

	console_render();
    
    /* Start the shell if the user presses start */
    printf("\n\nPress START to continue to the shell...\n");
    controller_init();
    while (true) {
        controller_scan();
        struct controller_data keys = get_keys_pressed();
        if (keys.c[0].start) {
            printf("Start pressed.\n");
            break;
        }
    }
    
    start_shell();
}
