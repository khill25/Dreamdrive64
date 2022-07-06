#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/pio.h"

#include "picocart64_pins.h"
// todo include some kind of data or rom file header

// Build cofig
// 0 = simple test program
// 1 = picocart64 ("master") pico 
// 2 = test pico that mimics n64 bus
// 3 = playground
// 4 = TEST -- SPI flash code
#define BUILD_CONFIG 4

#if BUILD_CONFIG == 3
#include "hardware/structs/systick.h"

/*
uint32_t startTime;
systick_hw->csr = 0x5;
systick_hw->rvr = 0x00FFFFFF;
startTime = systick_hw->cvr;
printf("read latency %d", systick_hw->cvr - startTime);
*/
int main() {
    stdio_init_all();

    sleep_ms(5000);

    systick_hw->csr = 0x5;
    systick_hw->rvr = 0x00FFFFFF;

    uint32_t new, old, t0, t1;
    old=systick_hw->cvr;

    t0=time_us_32();
    sleep_us(49999);
    new=systick_hw->cvr;
    t1=time_us_32();

    // printf("\n          old-new=%d\n",old-new);
    // printf("            t1-t0=%d\n",t1-t0);
    // printf("(old-new)/(t1-t0)=%.1f\n",(old-new)/(t1-t0*1.0));
    // printf("(t1-t0)/(old-new)=%.7f\n",1e3*(t1-t0)/(old-new));

    while(true) {
        
    }

    return 0;
}

#endif

#if BUILD_CONFIG == 0

#include "rom.h"

#define GPIO_0 0
#define GPIO_1 1
#define GPIO_2 2
#define GPIO_3 3

uint32_t responseStartTime = 0;
uint32_t responseTime = 0;
uint32_t buf[256];
#define DMA_TRANSFER_COUNT 256 // 256 transfers (of 32 bits)

// The XIP has some internal hardware that can stream a linear access sequence
// to a DMAable FIFO, while the system is still doing random accesses on flash
// code + data.
void read_from_flash() {
    // Start DMA transfer from XIP stream FIFO to our buffer in memory. Use
    // the auxiliary bus slave for the DMA<-FIFO accesses, to avoid stalling
    // the DMA against general XIP traffic. Doesn't really matter for this
    // example, but it can have a huge effect on DMA throughput.

    /// \tag::start_dma[]
    const uint dma_chan = 0;
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_XIP_STREAM);
    //channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16); // we can set the size of the data we are writing, probably would make sense to just write 16 bits and save half the data size
    dma_channel_configure(
            dma_chan,
            &cfg,
            (void *) buf,                 // Write addr
            (const void *) XIP_AUX_BASE,  // Read addr
            DMA_TRANSFER_COUNT,           // Transfer count
            false                         // Don't start immediately
    );
    /// \end::start_dma[]

    // Start a transfer
    dma_channel_transfer_from_buffer_now(dma_chan, (const void *) XIP_AUX_BASE, 1);

    dma_channel_wait_for_finish_blocking(dma_chan);

    // unsigned bit0 = (buf[0] >> 0) & 1; // get first 1 bit
    // // repeat for each bit (16)
    // // send bit to corresponding gpio pin
    // gpio_put(GPIO_0, bit0);

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
    for (int i = 0; i < DMA_TRANSFER_COUNT; ++i) {
        buf[i] = rom_file[i];
    }

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
#include "hardware/clocks.h"
#include "n64_data_control.pio.h"
#include "n64_data_tester.pio.h"
#include "rom.h"

#define CONTROL_SM 0
#define DATA_SM 0
PIO pio;
static const float pio_freq = 1024;//31250000;

#define BAD_DATA_ADDRESS 0
#define BAD_DATA_VALUE 1
#define BAD_DATA_ISBAD 2
uint32_t badData[256][3];
bool hasBadData = false;

static inline uint32_t swap16(uint32_t value) {
    // 0x11223344 => 0x33441122
    return (value << 16) | (value >> 16);
}

void sendAddress(uint32_t address) {
    pio_sm_put_blocking(pio, DATA_SM, swap16(address));
}

void verifyData(uint32_t data, uint32_t address, uint32_t index) {
    uint32_t rom_data = rom_file[(address & 0xFFFFFF) >> 1];

    if (data != rom_data) {
        badData[index][BAD_DATA_ADDRESS] = address;
        badData[index][BAD_DATA_VALUE] = data;
        badData[index][BAD_DATA_ISBAD] = 1;
        hasBadData = true;
    }
}

int main() {
    stdio_init_all();

    // init badData array
    for (int i = 0; i < 256; i++) {
        badData[i][BAD_DATA_ADDRESS] = 0;
        badData[i][BAD_DATA_VALUE] = 0;
        badData[i][BAD_DATA_ISBAD] = 0;
    }

    for (int i = 0; i <= 22; i++) {
        gpio_init(i);
        if (i < 16) {
            gpio_set_dir(i, GPIO_IN);
        } else {
            gpio_set_dir(i, GPIO_OUT);
        }
        gpio_pull_down(i);
    }

    for(int i = 0; i < 1000; i++) {
        gpio_put(N64_READ, true);
        gpio_put(N64_ALEH, true);
        gpio_put(N64_ALEL, true);
        sleep_ms(1);
        gpio_put(N64_READ, false);
        gpio_put(N64_ALEH, false);
        gpio_put(N64_ALEL, false);
        sleep_ms(1);
    }

    sleep_ms(1000);
    printf("Initializing...\n");

    // Init PIO
    pio = pio0;

    // Calculate the PIO clock divider
    float clockDivider = (float)clock_get_hz(clk_sys) / pio_freq;

    // data control state machine
    uint offset = pio_add_program(pio, &n64_data_control_program);
    n64_data_control_program_init(pio, CONTROL_SM, offset, clockDivider);
    
    // // data io state machine
    offset = pio_add_program(pio, &n64_data_tester_program);
    n64_data_tester_program_init(pio, DATA_SM, offset, clockDivider);

    // might need to add this for pios using `set`
    //pio_sm_set_set_pins(pio, 0, 16, 4);

    printf("Enabling PIO programs...\n");

    pio_sm_set_enabled(pio, CONTROL_SM, true);
    pio_sm_set_enabled(pio, DATA_SM, true);
    printf("PIO programs enabled!\n");

    uint32_t currentAddress = 0x10000000;//0x10000000;
    while(true) {
        // Now we can send the address that we want to read from
        sendAddress(currentAddress);
        currentAddress += 2;

        if (currentAddress > 0x40000000) {
            currentAddress = 0x10000000;
        }

        // // read data from bus until we have read 256 words of data (256 reads)
        // for (uint16_t i = 0; i < 256; i++) {
        //     sleep_ms(1000);
        //     gpio_put(N64_READ, true);
        //     sleep_ms(100);
        //     printf("waiting for data...\n");
        //     uint32_t data = pio_sm_get_blocking(pio, DATA_SM);
            
        //     // verify data
        //     verifyData(data, currentAddress, i);

        //     currentAddress += 2; // increment current address by 2 bytes each loop
            
        //     gpio_put(N64_READ, false);
        // }

        // if (hasBadData) {
        //     printf("Bad data detected\n\n\n");
        //     for (int i = 0; i < 256; i++) {
        //         if (badData[i][BAD_DATA_ISBAD] == 1) {
        //             printf("%#10x: %#04x, ", badData[i][BAD_DATA_ADDRESS], badData[i][BAD_DATA_VALUE]);
        //         }

        //         // while we are here, clear any state so we can use this again
        //         badData[i][BAD_DATA_ADDRESS] = 0;
        //         badData[i][BAD_DATA_VALUE] = 0;
        //         badData[i][BAD_DATA_ISBAD] = 0;
        //     }
        // } else {
        //     printf("No bad data detected!\n");
        // }

        // // reset the bad data flag as we go for another pass
        // hasBadData = false;

        // // restart both state machines so we can get more data for a different address offset (or just keep looping on the same data)
        // // pio_sm_restart(pio, CONTROL_SM);
        // pio_sm_restart(pio, DATA_SM);

        // // reset the current address, just keep looping over the same 256 words
        // currentAddress = 0x10000000;
    }

    return 0;
}

#endif

#if BUILD_CONFIG == 4
// Repurposed SPI flash read/write tests based on the pico/spi_flash example code
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/structs/systick.h"

#define FLASH_PAGE_SIZE        1024 * 8

#define FLASH_CMD_PAGE_PROGRAM 0x02
#define FLASH_CMD_READ         0x0B // read is 0x03, fast read is 0x0B, fast read has wait cycles between sending address and read
#define FLASH_CMD_STATUS       0x05
#define FLASH_CMD_WRITE_EN     0x06
#define FLASH_CMD_SECTOR_ERASE 0x20

#define FLASH_STATUS_BUSY_MASK 0x01

uint8_t page_buf[FLASH_PAGE_SIZE];

static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

void __not_in_flash_func(flash_read)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t *buf, size_t len) {
    cs_select(cs_pin);
    uint8_t cmdbuf[4] = {
            FLASH_CMD_READ,
            addr >> 16,
            addr >> 8,
            addr
    };
    spi_write_blocking(spi, cmdbuf, 4);
    spi_read_blocking(spi, 0, buf, len);
    cs_deselect(cs_pin);
}

void __not_in_flash_func(flash_write_enable)(spi_inst_t *spi, uint cs_pin) {
    cs_select(cs_pin);
    uint8_t cmd = FLASH_CMD_WRITE_EN;
    spi_write_blocking(spi, &cmd, 1);
    cs_deselect(cs_pin);
}

void __not_in_flash_func(flash_wait_done)(spi_inst_t *spi, uint cs_pin) {
    uint8_t status;
    do {
        cs_select(cs_pin);
        uint8_t buf[2] = {FLASH_CMD_STATUS, 0};
        spi_write_read_blocking(spi, buf, buf, 2);
        cs_deselect(cs_pin);
        status = buf[1];
    } while (status & FLASH_STATUS_BUSY_MASK);
}

void __not_in_flash_func(flash_sector_erase)(spi_inst_t *spi, uint cs_pin, uint32_t addr) {
    uint8_t cmdbuf[4] = {
            FLASH_CMD_SECTOR_ERASE,
            addr >> 16,
            addr >> 8,
            addr
    };
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 4);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}

void __not_in_flash_func(flash_page_program)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t data[]) {
    uint8_t cmdbuf[4] = {
            FLASH_CMD_PAGE_PROGRAM,
            addr >> 16,
            addr >> 8,
            addr
    };
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 4);
    spi_write_blocking(spi, data, FLASH_PAGE_SIZE);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}

void printbuf(uint8_t buf[FLASH_PAGE_SIZE]) {
    for (int i = 0; i < FLASH_PAGE_SIZE; ++i) {
        if (i % 16 == 15)
            printf("%02x\n", buf[i]);
        else
            printf("%02x ", buf[i]);
    }
}

void initSPI(uint32_t hz, bool verbose) {
    // Enable SPI 0 at 1 MHz and connect to GPIOs
    if (verbose) {
        printf("Initing SPI at %dMHZ\n", hz / 1000 / 1000);
    }
    spi_init(spi_default, hz);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));
}

uint32_t testRead(const uint32_t address, uint8_t *buf, uint32_t len) {
    uint32_t startTime = systick_hw->cvr;
    flash_read(spi_default, PICO_DEFAULT_SPI_CSN_PIN, address, buf, len);
    uint32_t totalTime = systick_hw->cvr - startTime;
    return totalTime;
}

// Generates random data to put into the buffer that will be written to flash
// Always writes the full size of page_buf[FLASG_PAGE_SIZE]
void testWrite(const uint32_t address) {
    // Writing new data to the buffer
    for (int i = 0; i < FLASH_PAGE_SIZE; ++i) {
        page_buf[i] = rand() % FLASH_PAGE_SIZE;
    }
    printf("Starting write...\n");
    uint32_t startTime = systick_hw->cvr;
    flash_page_program(spi_default, PICO_DEFAULT_SPI_CSN_PIN, address, page_buf);
    uint32_t totalTime = systick_hw->cvr - startTime;
    printf("write latency %d\n", totalTime);  
}

void testWriteRead(const uint32_t target_addr, uint32_t len) {
    testWrite(target_addr);
    
    printf("starting read...\n");
    uint32_t totalTime = testRead(target_addr, page_buf, len);
    printf("read latency %d\n", totalTime);
}

// Reads chunkSizeBytesToRead from flash
void runChunkReadTest(uint32_t speed, uint8_t testNumber, const uint32_t startingAddress, uint32_t chunkSizeBytesToRead) {
    printf("Chunk Read Test %d, %dMHZ, %d bytes\n", testNumber, speed / 1000 / 1000, chunkSizeBytesToRead);
    initSPI(speed, false);

    uint32_t readTime = 0;
    uint32_t currentAddress = startingAddress;
    uint8_t buf[chunkSizeBytesToRead];
    readTime = testRead(currentAddress, buf, chunkSizeBytesToRead);
    printf("latency %d cycles\n\n", readTime);

    // Rest between tests
    sleep_ms(100);
}

// This runs tests using the FLASH_PAGE_SIZE buffer
void runTest(uint32_t speed, uint8_t testNumber, const uint32_t target_addr, bool shouldPrintBuf) {
    printf("Test %d, %dMHZ\n", testNumber, speed / 1000 / 1000);
    initSPI(speed, false);

    // Write 0s to the buffer to 'erase' it just to make sure we aren't running into any weirdness
    for (int i = 0; i < FLASH_PAGE_SIZE; ++i) {
        page_buf[i] = 0;
    }

    flash_sector_erase(spi_default, PICO_DEFAULT_SPI_CSN_PIN, target_addr);
    if (shouldPrintBuf) {
        flash_read(spi_default, PICO_DEFAULT_SPI_CSN_PIN, target_addr, page_buf, FLASH_PAGE_SIZE);
        printf("Erased! contents:\n");
        printbuf(page_buf);
    }

    testWriteRead(target_addr, FLASH_PAGE_SIZE);

    printf("Write/Read complete! contents:\n");
    if (shouldPrintBuf) {
        printbuf(page_buf);
    }

    flash_sector_erase(spi_default, PICO_DEFAULT_SPI_CSN_PIN, target_addr);
    printf("-----------------Erased again! contents:\n");
    if (shouldPrintBuf) {
        flash_read(spi_default, PICO_DEFAULT_SPI_CSN_PIN, target_addr, page_buf, FLASH_PAGE_SIZE);
        printbuf(page_buf);
    }

    // Rest between tests
    sleep_ms(100);
}

int main() {
    // Enable UART so we can print status output
    stdio_init_all();
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_flash example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else

    sleep_ms(5000);

    printf("SPI flash speed test\n"); 
    // setup to use clock tick counting for more granular latency testing without an oscilloscope 
    systick_hw->csr = 0x5;
    systick_hw->rvr = 0x00FFFFFF;

    const uint32_t address = 0;
    bool shouldPrintBuf = false;
    uint32_t testSpeedsInMHZ[] = {16, 33, 66, 133};
    uint32_t testChunkByteSizes[] = {2, 8, 128, 512, 1024, 1024 * 2, 1024 * 4};
    for (int i = 0; i < count_of(testSpeedsInMHZ); i++) {
        uint32_t speed = 1000 * 1000 * testSpeedsInMHZ[i];
        //runTest(speed, i, address, shouldPrintBuf); // Run read/write tests
        for (int k = 0; k < count_of(testChunkByteSizes); k++) {
            runChunkReadTest(speed, i, address, testChunkByteSizes[k]);
        }
    }

    return 0;
#endif
}

#endif