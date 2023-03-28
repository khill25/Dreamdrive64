/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 * Copyright (c) 2022 Kaili Hill
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/structs/ssi.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/vreg.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mcu1.h"
#include "mcu2.h"
#include "qspi_helper.h"

#include "sdcard/internal_sd_card.h"

#define PIN_ID    (26)
#define MCU1_ID   ( 1)
#define MCU2_ID   ( 2)

bool g_isMCU1 = false;

/*
 * Dreamdrive64 is connected in the following way:
 * MCU1        - FLASH (+PSRAM) - MCU2
 * MCU1.GPIO26                  - 10k Pull Down
 * MCU2.GPIO26                  - 10k Pull Up
 * MCU2.GPIO19 - MCU1.RUN       - 10k Pull Down
 *
 * MCU2 boots first, copies program from flash to RAM and executes.
 * MCU2 reads GPIO26. If HIGH: we are MCU2.
 * MCU2 sets QSPI OutputEnable override to Disabled.
 * MCU2 drives GPIO19 HIGH, letting MCU1 boot.
 * MCU1 boots, copies program from flash to RAM and executes.
 * MCU1 reads GPIO26. If LOW: we are MCU1.
 * MCU1 sets QSPI OutputEnable override to Disabled.
 * Done.
 *
 */

// FreeRTOS boilerplate
void vApplicationGetTimerTaskMemory(StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t * pulTimerTaskStackSize)
{
	static StaticTask_t xTimerTaskTCB;
	static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

int main(void)
{
	// const int freq_khz = 133000;
	// const int freq_khz = 133000;
	// const int freq_khz = 140000;
	// const int freq_khz = 160000;
	// const int freq_khz = 180000;
	// const int freq_khz = 200000;
	// const int freq_khz = 210000;
	// const int freq_khz = 220000;
	// const int freq_khz = 230000;
	// const int freq_khz = 240000;
	// const int freq_khz = 250000;
	const int freq_khz = 266000;

	// Frequencies above 266MHz require extra voltage, or at least did with all the units I tested
	// const int freq_khz = 300000; 
	// const int freq_khz = 300000; 
	// const int freq_khz = 336000; // Boots and runs games with a 4x qspi divider
	// const int freq_khz = 360000; // Boots and runs games (at stock sdk speeds!) with a 4x qspi divider
	// const int freq_khz = 380000; // Boots and may run games with a 4x qspi divider
	// const int freq_khz = 480000; // Does not boot?
	// vreg_set_voltage(VREG_VOLTAGE_1_25);
	// vreg_set_voltage(VREG_VOLTAGE_1_15);
	bool clockWasSet = set_sys_clock_khz(freq_khz, false);

	// On MCU1, PIN_ID is pulled low externally.
	// On MCU2, PIN_ID is pulled high externally and connected to MCU1.RUN.
	gpio_init(PIN_ID);
	gpio_set_dir(PIN_ID, GPIO_IN);
	gpio_disable_pulls(PIN_ID);

	// mcu_id will be 1 for MCU1, and 2 for MCU2.
	int mcu_id = gpio_get(PIN_ID) + 1;
	DDR64_MCU_ID = mcu_id;

	if (mcu_id == MCU1_ID) {
		g_isMCU1 = true;

		mcu1_main();
	} else if (mcu_id == MCU2_ID) {
		g_isMCU1 = false;

		// Turn off SSI
		ssi_hw->ssienr = 0;
		// Disable output enable (OE) on all QSPI IO pins
		qspi_oeover_disable();
		
		// It's up to MCU2 to let MCU1 boot 
		mcu2_main();
	}
	// Never reached
	while (true) {
	}

	return 0;
}
