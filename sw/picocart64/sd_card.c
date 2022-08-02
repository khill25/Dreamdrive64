#include "sd_card.h"

static uint32_t sd_read_sector_start;
static uint32_t sd_read_sector_count;
static char sd_selected_rom_title[256];

void pc64_set_sd_read_sector(uint32_t sector) {
    sd_read_sector_start = sector;
}

void pc64_set_sd_read_sector_count(uint32_t count) {
    sd_read_sector_count = count;
}

void pc64_set_sd_rom_selection(char* title) {
    for (uint i = 0; i < 256; i++) {
        sd_selected_rom_title[i] = title[i];
    }
}

// Read data into scratch memory buffer
// buffer is 4k
// return data to cart
void pc64_read_sd(uint32_t sector, uint32_t count) {
    // over uart, ask second pico for info
    // wait for uart data
    // populate scratch memory buffer with response data
    
    // wait for all data to be returned?
    
    // In calling function:
    // return memory buffer data at address N, loop
}