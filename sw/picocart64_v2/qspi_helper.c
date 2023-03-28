#include "qspi_helper.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "stdio.h"
#include "utils.h"

#include "psram.h"
// #include "hardware/flash.h"
#include "hardware/resets.h" // pico-sdk reset defines
// #include "gpio_helper.h"

// #define VERBOSE

bool hasSavedValues = false;
uint32_t pad_values[NUM_QSPI_GPIOS];
uint32_t pad_pull_values[NUM_QSPI_GPIOS];
void qspi_print_pull(void)
{
	for (int i = 0; i < NUM_QSPI_GPIOS; i++) {
		printf("%d ORIG:%08x NOW:%08X\n", i, pad_values[i], pads_qspi_hw->io[i]);
	}

	printf("SS %08x ... %08x\n", sio_hw->gpio_hi_out, sio_hw->gpio_hi_out & (1u << 1u));
}

void qspi_restore_to_startup_config() {
	for (int i = 0; i < NUM_QSPI_GPIOS; i++) {
		pads_qspi_hw->io[i] = pad_values[i];
	}
}

void qspi_set_pull(bool disabled, bool pullup, bool pulldown)
{
	/*
	   Default bitmask:

	   0 00000021
	   1 00000050
	   2 00000050
	   3 00000050
	   4 00000050
	   5 0000005A
	 */

	pullup = false;
	pulldown = true;

	uint32_t values[6];
	uint32_t od = disabled ? 1UL : 0UL;
	uint32_t pue = pullup ? 1UL : 0UL;
	uint32_t pde = pulldown ? 1UL : 0UL;

	values[QSPI_SCLK_PAD] =
		(od << PADS_QSPI_GPIO_QSPI_SCLK_OD_LSB) |
		(1 << PADS_QSPI_GPIO_QSPI_SCLK_IE_LSB) |
		(3 << PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_LSB) |
		(1 << PADS_QSPI_GPIO_QSPI_SCLK_PUE_LSB) |
		(0 << PADS_QSPI_GPIO_QSPI_SCLK_PDE_LSB) | (0 << PADS_QSPI_GPIO_QSPI_SCLK_SCHMITT_LSB) | (0 << PADS_QSPI_GPIO_QSPI_SCLK_SLEWFAST_LSB);

	// Common values for SD0, SD1, SD2, SD3
	for (int i = 0; i < 4; i++) {
		values[QSPI_SD0_PAD + i] =
			(od << PADS_QSPI_GPIO_QSPI_SCLK_OD_LSB) |
			(1 << PADS_QSPI_GPIO_QSPI_SCLK_IE_LSB) |
			(0 << PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_LSB) |
			(0 << PADS_QSPI_GPIO_QSPI_SCLK_PUE_LSB) |
			(1 << PADS_QSPI_GPIO_QSPI_SCLK_PDE_LSB) | (0 << PADS_QSPI_GPIO_QSPI_SCLK_SCHMITT_LSB) | (0 << PADS_QSPI_GPIO_QSPI_SCLK_SLEWFAST_LSB);
	}

	values[QSPI_SS_PAD] =
		(od << PADS_QSPI_GPIO_QSPI_SCLK_OD_LSB) |
		(1 << PADS_QSPI_GPIO_QSPI_SCLK_IE_LSB) |
		(0 << PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_LSB) |
		(1 << PADS_QSPI_GPIO_QSPI_SCLK_PUE_LSB) |
		(0 << PADS_QSPI_GPIO_QSPI_SCLK_PDE_LSB) | (0 << PADS_QSPI_GPIO_QSPI_SCLK_SCHMITT_LSB) | (0 << PADS_QSPI_GPIO_QSPI_SCLK_SLEWFAST_LSB);

	for (int i = 0; i < NUM_QSPI_GPIOS; i++) {
		#if VERBOSE
			printf("%d %08X -> %08X\n", i, pads_qspi_hw->io[i], values[i]);
		#endif
		pads_qspi_hw->io[i] = values[i];
	}
}

void qspi_oeover_normal(bool enable_ss)
{
	ioqspi_hw->io[QSPI_SCLK_PIN].ctrl = (ioqspi_hw->io[QSPI_SCLK_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SCLK_CTRL_OEOVER_BITS) |
										 (IO_QSPI_GPIO_QSPI_SCLK_CTRL_OEOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SCLK_CTRL_OEOVER_LSB));

	if (enable_ss) {
		ioqspi_hw->io[QSPI_SS_PIN].ctrl = (ioqspi_hw->io[QSPI_SS_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS) |
										   (IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB));
	} else {
		ioqspi_hw->io[QSPI_SS_PIN].ctrl = (ioqspi_hw->io[QSPI_SS_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS) |
										   (IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_VALUE_DISABLE << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB));
	}

	ioqspi_hw->io[QSPI_SD0_PIN].ctrl = (ioqspi_hw->io[QSPI_SD1_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SD0_CTRL_OEOVER_BITS) |
										(IO_QSPI_GPIO_QSPI_SD0_CTRL_OEOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SD0_CTRL_OEOVER_LSB));

	ioqspi_hw->io[QSPI_SD1_PIN].ctrl = (ioqspi_hw->io[QSPI_SD2_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SD1_CTRL_OEOVER_BITS) |
										(IO_QSPI_GPIO_QSPI_SD1_CTRL_OEOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SD1_CTRL_OEOVER_LSB));

	ioqspi_hw->io[QSPI_SD2_PIN].ctrl = (ioqspi_hw->io[QSPI_SD3_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SD2_CTRL_OEOVER_BITS) |
										(IO_QSPI_GPIO_QSPI_SD2_CTRL_OEOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SD2_CTRL_OEOVER_LSB));

	ioqspi_hw->io[QSPI_SD3_PIN].ctrl = (ioqspi_hw->io[QSPI_SD3_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SD3_CTRL_OEOVER_BITS) |
										(IO_QSPI_GPIO_QSPI_SD3_CTRL_OEOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SD3_CTRL_OEOVER_LSB));

	qspi_set_pull(false, false, false);
}

void qspi_oeover_disable(void)
{

	if (!hasSavedValues) {
		for (int i = 0; i < NUM_QSPI_GPIOS; i++) {
			pad_values[i] = pads_qspi_hw->io[i];
		}
		hasSavedValues = true;
	}
	ioqspi_hw->io[QSPI_SCLK_PIN].ctrl = (ioqspi_hw->io[QSPI_SCLK_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SCLK_CTRL_OEOVER_BITS) |
										 (IO_QSPI_GPIO_QSPI_SCLK_CTRL_OEOVER_VALUE_DISABLE << IO_QSPI_GPIO_QSPI_SCLK_CTRL_OEOVER_LSB));

	// Set pull down on the clk pin. This will help with read errors on chip 1 and 2 
	// but doesn't eliminate them totally on rev a boards.
	hw_write_masked(
        &pads_qspi_hw->io[QSPI_SCLK_PIN],
        1 << PADS_QSPI_GPIO_QSPI_SCLK_PDE_LSB,
        PADS_QSPI_GPIO_QSPI_SCLK_PDE_BITS
    );

	ioqspi_hw->io[QSPI_SS_PIN].ctrl = (ioqspi_hw->io[QSPI_SS_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS) |
									   (IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_VALUE_DISABLE << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB));

	ioqspi_hw->io[QSPI_SD0_PIN].ctrl = (ioqspi_hw->io[QSPI_SD1_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SD0_CTRL_OEOVER_BITS) |
										(IO_QSPI_GPIO_QSPI_SD0_CTRL_OEOVER_VALUE_DISABLE << IO_QSPI_GPIO_QSPI_SD0_CTRL_OEOVER_LSB));

	ioqspi_hw->io[QSPI_SD1_PIN].ctrl = (ioqspi_hw->io[QSPI_SD2_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SD1_CTRL_OEOVER_BITS) |
										(IO_QSPI_GPIO_QSPI_SD1_CTRL_OEOVER_VALUE_DISABLE << IO_QSPI_GPIO_QSPI_SD1_CTRL_OEOVER_LSB));

	ioqspi_hw->io[QSPI_SD2_PIN].ctrl = (ioqspi_hw->io[QSPI_SD3_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SD2_CTRL_OEOVER_BITS) |
										(IO_QSPI_GPIO_QSPI_SD2_CTRL_OEOVER_VALUE_DISABLE << IO_QSPI_GPIO_QSPI_SD2_CTRL_OEOVER_LSB));

	ioqspi_hw->io[QSPI_SD3_PIN].ctrl = (ioqspi_hw->io[QSPI_SD3_PIN].ctrl & (~IO_QSPI_GPIO_QSPI_SD3_CTRL_OEOVER_BITS) |
										(IO_QSPI_GPIO_QSPI_SD3_CTRL_OEOVER_VALUE_DISABLE << IO_QSPI_GPIO_QSPI_SD3_CTRL_OEOVER_LSB));

	qspi_set_pull(true, false, true);
}

////////////////////////////////////////////////////////////
#define CMD_WRITE            		0x02
#define CMD_QUAD_WRITE       		0x38
#define CMD_READ_DATA        		0x03
#define CMD_FAST_READ       		0x0B
#define CMD_FAST_QUAD_READ   		0xEB
#define CMD_READ_STATUS      		0x05
#define PSRAM_ENTER_QUAD_MODE	  	0x35

#define SPI_DEFAULT_CLK_DIVIDER 	4 // when running in spi mode, divide sys clk by 4
////////////////////////////////////////////////////////////

static ssi_hw_t *const ssi = (ssi_hw_t *) XIP_SSI_BASE;

typedef enum {
    OUTOVER_NORMAL = 0,
    OUTOVER_INVERT,
    OUTOVER_LOW,
    OUTOVER_HIGH
} outover_t;

// Forward declarations
static void qspi_connect_xip_block();
static void qspi_cs_force(outover_t over);
static void qspi_exit_xip(int spi_clk_divider);
static void qspi_hard_abort();
static int qspi_was_hard_aborted();
static void qspi_flush_xip_cache();
static void qspi_init_spi(int clk_divider);

void qspi_enable(int clk_divider)
{
	qspi_oeover_normal(false); // want to use the CS line
	current_mcu_enable_demux(true);
	qspi_init_spi(clk_divider);
	// Turn off the XIP cache
	xip_ctrl_hw->ctrl = (xip_ctrl_hw->ctrl & (~XIP_CTRL_EN_BITS));
	xip_ctrl_hw->flush = 1;
}

void qspi_disable() {
	ssi_hw->ssienr = 0;
	qspi_oeover_disable();
	current_mcu_enable_demux(false);

// Pull down on qspi clk line. May help with errors on rev a, chip 1/2 when read from mcu1
	// hw_write_masked(
    //     &pads_qspi_hw->io[0],
    //     1 << PADS_QSPI_GPIO_QSPI_SCLK_PDE_LSB,
    //     PADS_QSPI_GPIO_QSPI_SCLK_PDE_BITS
    // );
}

// Setup ssi hardware in 1 bit SPI mode. Pass -1 for clk divider to use default.
void qspi_enable_spi(int clk_divider, int startingChipIndex) {
	int d = clk_divider == -1 ? SPI_DEFAULT_CLK_DIVIDER : clk_divider;
	qspi_enable(d);
    psram_set_cs(startingChipIndex); // Use the PSRAM chip
    qspi_connect_xip_block();
    qspi_exit_xip(d);
}

// Setup ssi hardware in 4 bit QSPI mode. Starting chip index is the first chip in array to send "enter quad mode" command to
// numChipsToEnable is the top chip number that quad mode should be enabled up to.
// Sets demux current chip to be `startingChipIndex`
void qspi_enable_qspi(int startingChipIndex, int lastChipIndex) {
	current_mcu_enable_demux(true);
    psram_set_cs(START_ROM_LOAD_CHIP_INDEX); // Use the PSRAM chip
    qspi_connect_xip_block();
    qspi_exit_xip(SPI_DEFAULT_CLK_DIVIDER);

	for(int i = startingChipIndex; i <= lastChipIndex; i++) {
        psram_set_cs(i);
        if(g_isMCU1) {
            //qspi_spi_do_cmd(0xC0, NULL, NULL, 0); // Toggle burst mode, doesn't seem to make much of a difference for higher frequencies?
        }
        sleep_ms(10);
        qspi_spi_do_cmd(0x35, NULL, NULL, 0);
        sleep_ms(50);    
    }

    qspi_flush_xip_cache();
	qspi_init_qspi();

    // change slew rate
    // for (int i = 0; i < 6; i++) {
    //     hw_write_masked(&pads_qspi_hw->io[i],
    //     (uint)0 << PADS_QSPI_GPIO_QSPI_SCLK_SLEWFAST_LSB,
    //     PADS_QSPI_GPIO_QSPI_SCLK_SLEWFAST_BITS
    //     );
    //     //PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_LSB
    //     //PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_BITS
    //     // printf("[%d] %08x\n", i, pads_qspi_hw->io[i]);
    // }

	psram_set_cs(startingChipIndex); // Use the PSRAM chip
}

void qspi_enable_flash(int clk_divider) {
	current_mcu_enable_demux(true);
	psram_set_cs(0);
	qspi_oeover_normal(true);

    // for (int i = 0; i < 6; i++) {
    //     hw_write_masked(&pads_qspi_hw->io[i],
    //     (uint)3 << PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_LSB,
    //     PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_BITS);
    //     // printf("[%d] %08x\n", i, pads_qspi_hw->io[i]);
    // }

// GPIO_DRIVE_STRENGTH_2MA = 0, ///< 2 mA nominal drive strength
// GPIO_DRIVE_STRENGTH_4MA = 1, ///< 4 mA nominal drive strength
// GPIO_DRIVE_STRENGTH_8MA = 2, ///< 8 mA nominal drive strength
// GPIO_DRIVE_STRENGTH_12MA = 3 ///< 12 mA nominal drive strength

	ssi_hw->ssienr = 0;
	ssi_hw->baudr = clk_divider; // change baud
    // ssi->rx_sample_dly = 4;  // 300-360
    // ssi->rx_sample_dly = 3;  // ??
    ssi->rx_sample_dly = 2;  // 266
    // ssi->rx_sample_dly = 0;  // ??
	ssi_hw->ssienr = 1;
}

// Set up the SSI controller for standard SPI mode,i.e. for every byte sent we get one back
// This is only called by flash_exit_xip(), not by any of the other functions.
// This makes it possible for the debugger or user code to edit SPI settings
// e.g. baud rate, CPOL/CPHA.
void qspi_init_spi(int clk_divider) {
	    // Disable SSI for further config
    ssi->ssienr = 0;
    // Clear sticky errors (clear-on-read)
    (void) ssi->sr;
    (void) ssi->icr;

	// CLK Divider. rp2040@266MHz div=4, qspi_clk=66MHz
    ssi->baudr = clk_divider;
    
	ssi->ctrlr0 =
            (SSI_CTRLR0_SPI_FRF_VALUE_STD << SSI_CTRLR0_SPI_FRF_LSB) | // Standard 1-bit SPI serial frames
            (7 << SSI_CTRLR0_DFS_32_LSB) | // 8 clocks per data frame
            (SSI_CTRLR0_TMOD_VALUE_TX_AND_RX << SSI_CTRLR0_TMOD_LSB);  // TX and RX FIFOs are both used for every byte
    // Slave selected when transfers in progress
    ssi->ser = 1;
    // Re-enable
    ssi->ssienr = 1;
}

// Helper method to setup the xip block
// Used before initing the ssi hardware for 1bit writes via 8bit data frames
static void qspi_connect_xip_block() {
    // Use hard reset to force IO and pad controls to known state (don't touch
    // IO_BANK0 as that does not affect XIP signals)
    // reset_unreset_block_wait_noinline(RESETS_RESET_IO_QSPI_BITS | RESETS_RESET_PADS_QSPI_BITS);
    reset_block(RESETS_RESET_IO_QSPI_BITS | RESETS_RESET_PADS_QSPI_BITS);
    unreset_block_wait(RESETS_RESET_IO_QSPI_BITS | RESETS_RESET_PADS_QSPI_BITS);

    // Then mux XIP block onto internal QSPI flash pads
    io_rw_32 *iobank1 = (io_rw_32 *) IO_QSPI_BASE;
#ifndef GENERAL_SIZE_HACKS
    for (int i = 0; i < 6; ++i)
        iobank1[2 * i + 1] = 0;
#else
    __asm volatile (
    "str %1, [%0, #4];" \
        "str %1, [%0, #12];" \
        "str %1, [%0, #20];" \
        "str %1, [%0, #28];" \
        "str %1, [%0, #36];" \
        "str %1, [%0, #44];"
    ::"r" (iobank1), "r" (0));
#endif
}

static void qspi_cs_force(outover_t over) {
	io_rw_32 *reg = (io_rw_32 *) (IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SS_CTRL_OFFSET);
#ifndef GENERAL_SIZE_HACKS
    *reg = *reg & ~IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS
        | (over << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB);
#else
    // The only functions we want are FSEL (== 0 for XIP) and OUTOVER!
    *reg = over << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB;
#endif
    // Read to flush async bridge
    (void) *reg;
}

// Put bytes from one buffer, and get bytes into another buffer.
// These can be the same buffer.
// If tx is NULL then send zeroes.
// If rx is NULL then all read data will be dropped.
//
// If rx_skip is nonzero, this many bytes will first be consumed from the FIFO,
// before reading a further count bytes into *rx.
// E.g. if you have written a command+address just before calling this function.
void qspi_spi_put_get(const uint8_t *tx, uint8_t *rx, size_t count, size_t rx_skip) {
	// Make sure there is never more data in flight than the depth of the RX
    // FIFO. Otherwise, when we are interrupted for long periods, hardware
    // will overflow the RX FIFO.
    const uint max_in_flight = 16 - 2; // account for data internal to SSI
    size_t tx_count = count;
    size_t rx_count = count;
    while (tx_count || rx_skip || rx_count) {
        // NB order of reads, for pessimism rather than optimism
        uint32_t tx_level = ssi_hw->txflr;
        uint32_t rx_level = ssi_hw->rxflr;
        bool did_something = false; // Expect this to be folded into control flow, not register
        if (tx_count && tx_level + rx_level < max_in_flight) {
            ssi->dr0 = (uint32_t) (tx ? *tx++ : 0);
            --tx_count;
            did_something = true;
        }
        if (rx_level) {
            uint8_t rxbyte = ssi->dr0;
            did_something = true;
            if (rx_skip) {
                --rx_skip;
            } else {
                if (rx)
                    *rx++ = rxbyte;
                --rx_count;
            }
        }
        // APB load costs 4 cycles, so only do it on idle loops (our budget is 48 cyc/byte)
        if (!did_something && __builtin_expect(qspi_was_hard_aborted(), 0))
            break;
    }
    qspi_cs_force(OUTOVER_HIGH);
}

void qspi_spi_do_cmd(uint8_t cmd, const uint8_t *tx, uint8_t *rx, size_t count) {
    qspi_cs_force(OUTOVER_LOW);
    ssi->dr0 = cmd;
    qspi_spi_put_get(tx, rx, count, 1);
}

void qspi_qspi_do_cmd(uint8_t cmd) {
    ssi->dr0 = cmd;
    int rx_skip = 1;
    uint8_t *tx = NULL;
    uint8_t *rx = NULL;
    int count = 0;
    //program_flash_put_get(NULL, NULL, 0, 1);
    // Make sure there is never more data in flight than the depth of the RX
    // FIFO. Otherwise, when we are interrupted for long periods, hardware
    // will overflow the RX FIFO.
    const uint max_in_flight = 16 - 2; // account for data internal to SSI
    size_t tx_count = count;
    size_t rx_count = count;
    while (tx_count || rx_skip || rx_count) {
        // NB order of reads, for pessimism rather than optimism
        uint32_t tx_level = ssi_hw->txflr;
        uint32_t rx_level = ssi_hw->rxflr;
        bool did_something = false; // Expect this to be folded into control flow, not register
        if (tx_count && tx_level + rx_level < max_in_flight) {
            ssi->dr0 = (uint32_t) (tx ? *tx++ : 0);
            --tx_count;
            did_something = true;
        }
        if (rx_level) {
            uint8_t rxbyte = ssi->dr0;
            did_something = true;
            if (rx_skip) {
                --rx_skip;
            } else {
                if (rx)
                    *rx++ = rxbyte;
                --rx_count;
            }
        }
        // APB load costs 4 cycles, so only do it on idle loops (our budget is 48 cyc/byte)
        // if (!did_something && __builtin_expect(program_flash_was_aborted(), 0))
        if (!did_something) {
           break;
        }
    }
}

// Used to exit quad mode on the current psram chip. 
// Meant for qspi in qspi mode, not spi.
// Safe to call multiple times on the same chip. 
// Will have no effect after the first time as the chip won't recognize the quad commands being sent.
void qspi_qspi_exit_quad_mode() {
	ssi->dr0 = 0xF5;
    int rx_skip = 1;
    uint8_t *tx = NULL;
    uint8_t *rx = NULL;
    int count = 0;
    //program_flash_put_get(NULL, NULL, 0, 1);
    // Make sure there is never more data in flight than the depth of the RX
    // FIFO. Otherwise, when we are interrupted for long periods, hardware
    // will overflow the RX FIFO.
    const uint max_in_flight = 16 - 2; // account for data internal to SSI
    size_t tx_count = count;
    size_t rx_count = count;
    while (tx_count || rx_skip || rx_count) {
        // NB order of reads, for pessimism rather than optimism
        uint32_t tx_level = ssi_hw->txflr;
        uint32_t rx_level = ssi_hw->rxflr;
        bool did_something = false; // Expect this to be folded into control flow, not register
        if (tx_count && tx_level + rx_level < max_in_flight) {
            ssi->dr0 = (uint32_t) (tx ? *tx++ : 0);
            --tx_count;
            did_something = true;
        }
        if (rx_level) {
            uint8_t rxbyte = ssi->dr0;
            did_something = true;
            if (rx_skip) {
                --rx_skip;
            } else {
                if (rx)
                    *rx++ = rxbyte;
                --rx_count;
            }
        }
        // APB load costs 4 cycles, so only do it on idle loops (our budget is 48 cyc/byte)
        // if (!did_something && __builtin_expect(program_flash_was_aborted(), 0))
        if (!did_something) {
           break;
        }
    }
}

void qspi_spi_put_cmd_addr(uint8_t cmd, uint32_t addr) {
	qspi_cs_force(OUTOVER_LOW);
    addr |= cmd << 24;
    for (int i = 0; i < 4; ++i) {
        ssi->dr0 = addr >> 24;
        addr <<= 8;
    }
}

// GCC produces some heinous code if we try to loop over the pad controls,
// so structs it is
struct sd_padctrl {
    io_rw_32 sd0;
    io_rw_32 sd1;
    io_rw_32 sd2;
    io_rw_32 sd3;
};

// Calls qspi_init_spi() as part of exiting xip
// Sequence:
// 1. CSn = 1, IO = 4'h0 (via pulldown to avoid contention), x32 clocks
// 2. CSn = 0, IO = 4'hf (via pullup to avoid contention), x32 clocks
// 3. CSn = 1 (brief deassertion)
// 4. CSn = 0, MOSI = 1'b1 driven, x16 clocks
//
// Part 4 is the sequence suggested in W25X10CL datasheet.
// Parts 1 and 2 are to improve compatibility with Micron parts
static void qspi_exit_xip(int spi_clk_divider) {
struct sd_padctrl *qspi_sd_padctrl = (struct sd_padctrl *) (PADS_QSPI_BASE + PADS_QSPI_GPIO_QSPI_SD0_OFFSET);
    io_rw_32 *qspi_ss_ioctrl = (io_rw_32 *) (IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SS_CTRL_OFFSET);
    uint8_t buf[2];
    buf[0] = 0xff;
    buf[1] = 0xff;

    qspi_init_spi(spi_clk_divider);

    uint32_t padctrl_save = qspi_sd_padctrl->sd0;
    uint32_t padctrl_tmp = (padctrl_save
                            & ~(PADS_QSPI_GPIO_QSPI_SD0_OD_BITS | PADS_QSPI_GPIO_QSPI_SD0_PUE_BITS |
                                PADS_QSPI_GPIO_QSPI_SD0_PDE_BITS)
                           ) | PADS_QSPI_GPIO_QSPI_SD0_OD_BITS | PADS_QSPI_GPIO_QSPI_SD0_PDE_BITS;

    // First two 32-clock sequences
    // CSn is held high for the first 32 clocks, then asserted low for next 32
    qspi_cs_force(OUTOVER_HIGH);
    for (int i = 0; i < 2; ++i) {
        // This gives 4 16-bit offset store instructions. Anything else seems to
        // produce a large island of constants
        qspi_sd_padctrl->sd0 = padctrl_tmp;
        qspi_sd_padctrl->sd1 = padctrl_tmp;
        qspi_sd_padctrl->sd2 = padctrl_tmp;
        qspi_sd_padctrl->sd3 = padctrl_tmp;

        // Brief delay (~6000 cyc) for pulls to take effect
        uint32_t delay_cnt = 1u << 11;
        asm volatile (
        "1: \n\t"
        "sub %0, %0, #1 \n\t"
        "bne 1b"
        : "+r" (delay_cnt)
        );

        qspi_spi_put_get(NULL, NULL, 4, 0);

        padctrl_tmp = (padctrl_tmp
                       & ~PADS_QSPI_GPIO_QSPI_SD0_PDE_BITS)
                      | PADS_QSPI_GPIO_QSPI_SD0_PUE_BITS;

        qspi_cs_force(OUTOVER_LOW);
    }

    // Restore IO/pad controls, and send 0xff, 0xff. Put pullup on IO2/IO3 as
    // these may be used as WPn/HOLDn at this point, and we are now starting
    // to issue serial commands.

    qspi_sd_padctrl->sd0 = padctrl_save;
    qspi_sd_padctrl->sd1 = padctrl_save;
    padctrl_save = (padctrl_save
        & ~PADS_QSPI_GPIO_QSPI_SD0_PDE_BITS
    ) | PADS_QSPI_GPIO_QSPI_SD0_PUE_BITS;
    qspi_sd_padctrl->sd2 = padctrl_save;
    qspi_sd_padctrl->sd3 = padctrl_save;

    qspi_cs_force(OUTOVER_LOW);
    qspi_spi_put_get(buf, NULL, 2, 0);

    *qspi_ss_ioctrl = 0;
}

void qspi_spi_wait_ready() {
	uint8_t stat;
    do {
        qspi_spi_do_cmd(CMD_READ_STATUS, NULL, &stat, 1);
    } while (stat & 0x1 && !qspi_was_hard_aborted());
}

void qspi_spi_write_buf(uint32_t addr, const uint8_t* data, uint32_t len) {
    qspi_spi_put_cmd_addr(CMD_WRITE, addr);
    qspi_spi_put_get(data, NULL, len, 4);
}

// Force MISO input to SSI low so that an in-progress SR polling loop will
// fall through. This is needed when a flash programming task in async task
// context is locked up (e.g. if there is no flash device, and a hard pullup
// on MISO pin -> SR read gives 0xff) and the host issues an abort in IRQ
// context. Bit of a hack
static void qspi_hard_abort() {
    hw_set_bits(
            (io_rw_32 *) (IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SD1_CTRL_OFFSET),
            IO_QSPI_GPIO_QSPI_SD1_CTRL_INOVER_VALUE_LOW << IO_QSPI_GPIO_QSPI_SD1_CTRL_INOVER_LSB
    );
}

// Also allow any unbounded loops to check whether the above abort condition
// was asserted, and terminate early
static int qspi_was_hard_aborted() {
    return *(io_rw_32 *) (IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SD1_CTRL_OFFSET)
           & IO_QSPI_GPIO_QSPI_SD1_CTRL_INOVER_BITS;
}

// ----------------------------------------------------------------------------
// Read

void qspi_spi_read_data(uint32_t addr, uint8_t *rx, size_t count) {
    assert(addr < 0x1000000);
    qspi_spi_put_cmd_addr(CMD_READ_DATA, addr);
    qspi_spi_put_get(NULL, rx, count, 4);
}

// ----------------------------------------------------------------------------
// XIP Entry

// This is a hook for steps to be taken in between programming the flash and
// doing cached XIP reads from the flash. Called by the bootrom before
// entering flash second stage, and called by the debugger after flash
// programming.
static void qspi_flush_xip_cache() {
    xip_ctrl_hw->flush = 1;
    // Read blocks until flush completion
    (void) xip_ctrl_hw->flush;
    // Enable the cache
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_EN_BITS);
    qspi_cs_force(OUTOVER_NORMAL);
}

// Must be called while the ssi hardware it setup for spi
// Sends an "enter quad mode" command to current psram chip
// and sets up the hardware to read with Quad fast read commands.
void qspi_init_qspi() {
    ssi->ssienr = 0;
    ssi->baudr = QSPI_QUAD_MODE_CLK_DIVIDER;
    ssi->ctrlr0 =
            (SSI_CTRLR0_SPI_FRF_VALUE_QUAD << SSI_CTRLR0_SPI_FRF_LSB) |  // Quad SPI serial frames
            (31 << SSI_CTRLR0_DFS_32_LSB) |                             // 32 clocks per data frame
            (SSI_CTRLR0_TMOD_VALUE_EEPROM_READ << SSI_CTRLR0_TMOD_LSB); // Send instr + addr, receive data
    ssi->spi_ctrlr0 =
            (0xEB << SSI_SPI_CTRLR0_XIP_CMD_LSB) | 
            (6u << SSI_SPI_CTRLR0_WAIT_CYCLES_LSB) |
            (SSI_SPI_CTRLR0_INST_L_VALUE_8B << SSI_SPI_CTRLR0_INST_L_LSB) |    // 
            (6u << SSI_SPI_CTRLR0_ADDR_L_LSB) |    // 24-bit addressing for 03h commands
            (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_2C2A  // Command and address both in serial format
                    << SSI_SPI_CTRLR0_TRANS_TYPE_LSB);

	// ssi->rx_sample_dly = 3;
    ssi->rx_sample_dly = 2;
    // ssi->rx_sample_dly = 1;
    ssi->ssienr = 1;
}

// LEAVE THIS METHOD FOR LEGACY REASONS
// Put the SSI into a mode where XIP accesses translate to standard
// serial 03h read commands. The flash remains in its default serial command
// state, so will still respond to other commands.
void __noinline program_flash_enter_cmd_xip(bool isPSRAM) {

    // for (int i = 0; i < 6; i++) {
    //     hw_write_masked(&pads_qspi_hw->io[i],
    //     (uint)1 << PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_LSB,
    //     PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_BITS);
    //     // printf("[%d] %08x\n", i, pads_qspi_hw->io[i]);
    // }

// GPIO_DRIVE_STRENGTH_2MA = 0, ///< 2 mA nominal drive strength
// GPIO_DRIVE_STRENGTH_4MA = 1, ///< 4 mA nominal drive strength
// GPIO_DRIVE_STRENGTH_8MA = 2, ///< 8 mA nominal drive strength
// GPIO_DRIVE_STRENGTH_12MA = 3 ///< 12 mA nominal drive strength

    if (isPSRAM) {
        qspi_init_qspi();
    } else {

    // // DEFAULT THAT WAS HERE
    // ssi->ssienr = 0;
    // ssi->ctrlr0 =
    //         (SSI_CTRLR0_SPI_FRF_VALUE_STD << SSI_CTRLR0_SPI_FRF_LSB) |  // Standard 1-bit SPI serial frames
    //         (31 << SSI_CTRLR0_DFS_32_LSB) |                             // 32 clocks per data frame
    //         (SSI_CTRLR0_TMOD_VALUE_EEPROM_READ << SSI_CTRLR0_TMOD_LSB); // Send instr + addr, receive data
    // ssi->spi_ctrlr0 =
    //         (FLASHCMD_READ_DATA << SSI_SPI_CTRLR0_XIP_CMD_LSB) | // Standard 03h read
    //         (2u << SSI_SPI_CTRLR0_INST_L_LSB) |    // 8-bit instruction prefix
    //         (6u << SSI_SPI_CTRLR0_ADDR_L_LSB) |    // 24-bit addressing for 03h commands
    //         (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_1C1A  // Command and address both in serial format
    //                 << SSI_SPI_CTRLR0_TRANS_TYPE_LSB);
    // ssi->ssienr = 1;

// THIS WORKS!
    // ssi->ssienr = 0;
    // ssi->baudr = 2;
    // ssi->ctrlr0 =
    //         (SSI_CTRLR0_SPI_FRF_VALUE_QUAD << SSI_CTRLR0_SPI_FRF_LSB) |  // Standard 1-bit SPI serial frames
    //         (31 << SSI_CTRLR0_DFS_32_LSB) |                             // 32 clocks per data frame
    //         (SSI_CTRLR0_TMOD_VALUE_EEPROM_READ << SSI_CTRLR0_TMOD_LSB); // Send instr + addr, receive data
    // ssi->spi_ctrlr0 =
    //         (0x6B << SSI_SPI_CTRLR0_XIP_CMD_LSB) | // Standard 03h read
    //         (8u << SSI_SPI_CTRLR0_WAIT_CYCLES_LSB) |
    //         (2u << SSI_SPI_CTRLR0_INST_L_LSB) |    // 8-bit instruction prefix
    //         (6u << SSI_SPI_CTRLR0_ADDR_L_LSB) |    // 24-bit addressing for 03h commands
    //         (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_1C1A  // Command and address both in serial format
    //                 << SSI_SPI_CTRLR0_TRANS_TYPE_LSB);
    // ssi->ssienr = 1;


// FLASH QUAD MODE XIP - works
    // printf("Configure xip...\n");
    ssi->ssienr = 0;
    ssi->baudr = 4;
    ssi->ctrlr0 =
            (SSI_CTRLR0_SPI_FRF_VALUE_QUAD << SSI_CTRLR0_SPI_FRF_LSB) |  // Standard 1-bit SPI serial frames
            (31 << SSI_CTRLR0_DFS_32_LSB) |                             // 32 clocks per data frame
            (SSI_CTRLR0_TMOD_VALUE_EEPROM_READ << SSI_CTRLR0_TMOD_LSB); // Send instr + addr, receive data
    ssi->spi_ctrlr0 =
            (4u << SSI_SPI_CTRLR0_WAIT_CYCLES_LSB) |
            (2u << SSI_SPI_CTRLR0_INST_L_LSB) |    // 8-bit instruction prefix
            (8u << SSI_SPI_CTRLR0_ADDR_L_LSB) |    // 24-bit addressing for 03h commands
            (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_1C2A  // Command in serial format address in quad
                    << SSI_SPI_CTRLR0_TRANS_TYPE_LSB);
    ssi->ssienr = 1;

    ssi->dr0 = 0xEB;
    ssi->dr0 = 0x000000a0;

    while ((ssi_hw->sr & SSI_SR_BUSY_BITS) != 0) { tight_loop_contents(); }  

    ssi->ssienr = 0;
    ssi->baudr = 2;
    ssi->ctrlr0 =
            (SSI_CTRLR0_SPI_FRF_VALUE_QUAD << SSI_CTRLR0_SPI_FRF_LSB) |  // Standard 1-bit SPI serial frames
            (31 << SSI_CTRLR0_DFS_32_LSB) |                             // 32 clocks per data frame
            (SSI_CTRLR0_TMOD_VALUE_EEPROM_READ << SSI_CTRLR0_TMOD_LSB); // Send instr + addr, receive data
    ssi->spi_ctrlr0 =
            (0xa0 << SSI_SPI_CTRLR0_XIP_CMD_LSB) | // Standard 03h read
            (4u << SSI_SPI_CTRLR0_WAIT_CYCLES_LSB) |
            (SSI_SPI_CTRLR0_INST_L_VALUE_NONE << SSI_SPI_CTRLR0_INST_L_LSB) |    // 
            (8u << SSI_SPI_CTRLR0_ADDR_L_LSB) |    // 24-bit addressing for 03h commands
            (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_2C2A  // Command and address both in serial format
                    << SSI_SPI_CTRLR0_TRANS_TYPE_LSB);
    ssi->ssienr = 1;
    }

    // printf("DONE!\n");
}






