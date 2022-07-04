#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/pio.h"

#include "picocart64_pins.h"
#include "data.h"

// Build cofig
// 0 = simple test program
// 1 = picocart64 ("master") pico 
// 2 = test pico that mimics n64 bus
#define BUILD_CONFIG 2

#if BUILD_CONFIG == 0
#define GPIO_0 0
#define GPIO_1 1
#define GPIO_2 2
#define GPIO_3 3

uint32_t responseStartTime = 0;
uint32_t responseTime = 0;
uint32_t buf[count_of(data)];

// The XIP has some internal hardware that can stream a linear access sequence
// to a DMAable FIFO, while the system is still doing random accesses on flash
// code + data.
void trigger_data_write() {
    // Start DMA transfer from XIP stream FIFO to our buffer in memory. Use
    // the auxiliary bus slave for the DMA<-FIFO accesses, to avoid stalling
    // the DMA against general XIP traffic. Doesn't really matter for this
    // example, but it can have a huge effect on DMA throughput.

    // printf("Starting DMA\n");
    /// \tag::start_dma[]
    const uint dma_chan = 0;
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_XIP_STREAM);
    //channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    dma_channel_configure(
            dma_chan,
            &cfg,
            (void *) buf,                 // Write addr
            (const void *) XIP_AUX_BASE,  // Read addr
            count_of(data),               // Transfer count
            false                         // Don't start immediately
    );
    /// \end::start_dma[]

    // Start a transfer
    dma_channel_transfer_from_buffer_now(dma_chan, (const void *) XIP_AUX_BASE, 1);

    dma_channel_wait_for_finish_blocking(dma_chan);

    // printf("DMA complete\n");

    unsigned bit0 = (buf[0] >> 0) & 1; // get first 1 bit
    // repeat for each bit (16)
    // send bit to corresponding gpio pin
    gpio_put(GPIO_0, bit0);

    responseTime = time_us_32() - responseStartTime;
    printf("response time %zu for value %d\n", responseTime, bit0);
    responseStartTime = 0;
    responseTime = 0;
}

void buttonPressed() {
    printf("button pressed\n");
    responseStartTime = time_us_32();
    trigger_data_write();
}

void gpio_callback(uint gpio, uint32_t events) {   
    buttonPressed();
}

int writeDataToFlash() {
    printf("writing data to flash\n");
    for (int i = 0; i < count_of(data); ++i)
        buf[i] = 0;

    // This example won't work with PICO_NO_FLASH builds. Note that XIP stream
    // can be made to work in these cases, if you enable some XIP mode first
    // (e.g. via calling flash_enter_cmd_xip() in ROM). However, you will get
    // much better performance by DMAing directly from the SSI's FIFOs, as in
    // this way you can clock data continuously on the QSPI bus, rather than a
    // series of short transfers.
    if ((uint32_t) &data[0] >= SRAM_BASE) {
        printf("You need to run this example from flash!\n");
        return -1;
    }

    // Transfer started by writing nonzero value to stream_ctr. stream_ctr
    // will count down as the transfer progresses. Can terminate early by
    // writing 0 to stream_ctr.
    // It's a good idea to drain the FIFO first!

    /// \tag::start_stream[]
    while (!(xip_ctrl_hw->stat & XIP_STAT_FIFO_EMPTY))
        (void) xip_ctrl_hw->stream_fifo;
    xip_ctrl_hw->stream_addr = (uint32_t) &data[0];
    xip_ctrl_hw->stream_ctr = count_of(data);
    /// \end::start_stream[]

    return 0;
}

int main() {
    stdio_init_all();

    // setup gpio
    gpio_init(GPIO_0);
    gpio_set_dir(GPIO_0, GPIO_OUT);
    
    gpio_init(GPIO_1);
    gpio_set_dir(GPIO_1, GPIO_OUT);

    gpio_init(GPIO_2);
    gpio_set_dir(GPIO_2, GPIO_IN);
    gpio_pull_down(GPIO_2);
    
    gpio_init(GPIO_3);
    gpio_set_dir(GPIO_3, GPIO_IN);

    // Setup an inturrupt when we press a button to simulate a signal on this gpio pin
    // This isn't working :(
    //gpio_set_irq_enabled_with_callback(GPIO_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // write test data to flash memory
    int errorCode = writeDataToFlash();

    if (errorCode == -1) {
        printf("Error! See logs for details.\n");
        return errorCode;
    }

    // main loop
    uint32_t t = 0;
    uint32_t lastCheck = 0;
    bool button = gpio_get(GPIO_2);
    while (1) {
        uint32_t now = time_us_32();
        t += now - lastCheck;

        // if (t > 1000000) {
        //     printf("heartbeat - %d seconds\n", now / 1000000);
        //     printf("pressed %d, low %d\n", gpio_get(GPIO_2), gpio_is_pulled_down(GPIO_2));
        //     t = 0;
        // }

        if (button != gpio_get(GPIO_2)) {
            buttonPressed();
            button = !button;
        }
        
        lastCheck = now;
    }

    return 0;
}
#endif

/*
 * Test harness to run on second pico (pico1)
 * reads and writes data across all the bus pins like it's the n64
 * 
 * Verifies data sent back and logs to serial output
 * 
 * ? Should this also emulate the n64 clock pins? (ALE_L, ALE_H)?
 */
#if BUILD_CONFIG == 2
#include "n64_data_control.pio.h"
#include "n64_data_tester.pio.h"

PIO pio;

static inline uint32_t swap16(uint32_t value)
{
    // 0x11223344 => 0x33441122
    return (value << 16) | (value >> 16);
}

void sendAddress(uint32_t address) {
    pio_sm_put_blocking(pio, 0, swap16(address));
}

int main() {
    stdio_init_all();

    for (int i = 0; i <= 22; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_pulls(i, false, false);
    }

    // Init PIO
    pio = pio0;

    // data control state machine
    uint offset = pio_add_program(pio, &n64_data_control_program);
    n64_data_control_program_init(pio, 1, offset);
    pio_sm_set_enabled(pio, 1, true);

    // data io state machine
    offset = pio_add_program(pio, &n64_data_tester_program);
    n64_data_tester_program_init(pio, 0, offset);
    pio_sm_set_enabled(pio, 0, true);

    uint32_t currentAddress = 0x10000000;
    while(true) {
        // Now we can send the address that we want to read from
        sendAddress(currentAddress);

        // read data from bus until we have read 256 words of data (256 reads)
        for (uint16_t i = 0; i < 256; i++) {
            uint32_t data = pio_sm_get_blocking(pio, 0);
            // verify data
        }

        // restart both state machines so we can get more data for a different address offset (or just keep looping on the same data)
        pio_sm_restart(pio, 0);
        pio_sm_restart(pio, 1);
    }

    return 0;
}

void writeData() {

}

void readData() {

}

#endif

// Picocart64 code (this would be the pico connected to the n64 bus)
#if BUILD_CONFIG == 1
#include "n64_pi.pio.h"
//const uint16_t *rom_file_16 = (uint16_t *) rom_file;

uint32_t SRAM[32 * 1024 / 4];
uint16_t *SRAM_16 = (uint16_t *) SRAM;

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

#define PICO_LA1    (26)
#define PICO_LA2    (27)

#define UART_TX_PIN (28)
#define UART_RX_PIN (29) /* not available on the pico */
#define UART_ID     uart0
#define BAUD_RATE   115200


static inline uint32_t swap16(uint32_t value)
{
    // 0x11223344 => 0x33441122
    return (value << 16) | (value >> 16);
}

static inline uint32_t swap8(uint16_t value)
{
    // 0x1122 => 0x2211
    return (value << 8) | (value >> 8);
}

int main() {
    // Overclock!
    // Note that the Pico's external flash is rated to 133MHz,
    // not sure if the flash speed is based on this clock.

    // set_sys_clock_khz(PLL_SYS_KHZ, true);
    // set_sys_clock_khz(150000, true); // Does not work
    // set_sys_clock_khz(200000, true); // Does not work
    // set_sys_clock_khz(250000, true); // Does not work
    // set_sys_clock_khz(300000, true); // Doesn't even boot
    // set_sys_clock_khz(400000, true); // Doesn't even boot

    // stdio_init_all();

    for (int i = 0; i <= 27; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_pulls(i, false, false);
    }

    gpio_init(N64_CIC_DCLK);
    gpio_init(N64_CIC_DIO);
    gpio_init(N64_COLD_RESET);

    gpio_pull_up(N64_CIC_DIO);

    // Init UART on pin 28/29
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    printf("PicoCart64 Booting!\r\n");

    // Init PIO before starting the second core
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &n64_pi_program);
    n64_pi_program_init(pio, 0, offset);
    pio_sm_set_enabled(pio, 0, true);

    // Launch the CIC emulator in the second core
    // Note! You have to power reset the pico after flashing it with a jlink,
    //       otherwise multicore doesn't work properly.
    //       Alternatively, attach gdb to openocd, run `mon reset halt`, `c`.
    //       It seems this works around the issue as well.
    //multicore_launch_core1(cic_main);

    // Wait for reset to be released
    while (gpio_get(N64_COLD_RESET) == 0) {
        tight_loop_contents();
    }

    uint32_t n64_addr = 0;
    uint32_t n64_addr_h = 0;
    uint32_t n64_addr_l = 0;

    uint32_t last_addr = 0;
    uint32_t get_msb = 0;

    while (1) {
        uint32_t addr = swap16(pio_sm_get_blocking(pio, 0));

        if (addr & 0x00000001) {
            // We got a WRITE
            // 0bxxxxxxxx_xxxxxxxx_11111111_11111111
            SRAM_16[(last_addr & 0xFFFFFF)>>1] = addr >> 16;
            last_addr += 2;
            continue;
        }

        if (addr != 0) {
            // We got a start address
            last_addr = addr;
            get_msb = 1;
            continue;
        }

        // We got a "Give me next 16 bits" command
        uint32_t word;
        if (last_addr == 0x10000000) {
            // Configure bus to run slowly.
            // This is better patched in the rom, so we won't need a branch here.
            // But let's keep it here so it's easy to import roms easily.
            // 0x8037FF40 in big-endian
            word = 0x8037;
            pio_sm_put_blocking(pio, 0, word);
        } else if (last_addr == 0x10000002) {
            // Configure bus to run slowly.
            // This is better patched in the rom, so we won't need a branch here.
            // But let's keep it here so it's easy to import roms easily.
            // 0x8037FF40 in big-endian
            word = 0xFF40;
            pio_sm_put_blocking(pio, 0, word);
        } else if (last_addr >= 0x08000000 && last_addr <= 0x0FFFFFFF) {
            // Domain 2, Address 2	Cartridge SRAM
            word = SRAM_16[(last_addr & 0xFFFFFF) >> 1];
            pio_sm_put_blocking(pio, 0, word);
        } else if (last_addr >= 0x10000000 && last_addr <= 0x1FBFFFFF) {
            // Domain 1, Address 2	Cartridge ROM

            // Since I don't have the rom.h file, just comment out for the time being
            // word = rom_file_16[(last_addr & 0xFFFFFF) >> 1];
            // pio_sm_put_blocking(pio, 0, swap8(word));
        }

        last_addr += 2;
    }

    return 0;
}
#endif