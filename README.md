# DREAMDrive64
Nintendo 64 flash cart based around Raspberry Pi RP2040s. Load roms up to 64MB, EEPROM save support, supports all cic versions.

---
# Getting one
## Buy one from my website! 
https://dreamcraftindustries.com/products/dreamdrive64

## Build your own with PCBWay
You can load up the hw files in Kicad and use the PCBWay plugin. The PCBWay plugin will take you right to the PCBWay website with the gerbers loaded up and ready to order. 

### N64 Cartridge Shell
You can use a doner shell or find some on Aliexpress/Ebay.
The shell will need to be cut to accomodate the SD Card slot and USB port.

* SD Card Slot Cuts 
	* These cuts should be 15mm wide
		* This cut should start approximately 46.5mm from the bottom of the shell on the outside edge.
	* 3mm Deep on the back shell
	* Less than 1mm deep on the front shell
	
* "Boot" Button (Press while plugging in USB to load new firmware)
	* Drill small hole
		* 28.6mm from left side of back shell (5mm right of screw hole)
		* ~6.2mm down from the screw hole

---
# Current Dev Status
* DREAMDrive64 can boot roms and has made the v1.0.0 release! 
	* Menu rom can navigate the sd file system assuming a rational number of files/folders (256 at the moment per directory, 10 directory depth limit)
	* Menu rom can render thumbnail boxart of a rom when scrolling through file list
	* Libdragon changes are needed if you want to compile your own rom for the firmware, those changes are in PR over at the libdragon repo
	* Firmware is stable with a v1.0.0 release available right now, with more improvements on the way.
 

## Features
| Feature					| Status 	 | Notes 	|
|--------------------------:|:-----------|---------:|
| Rom Loading				| Complete!	 |			|
| SD Filesystem navigation	| Complete!	 | Folder/File limit ~100 per directory 		|
| Wifi						| Soon		 |			|
| EEPROM save support		| Complete!		 |			|
| SRAM support 				| Complete!  | Some games require much higher sram timing than is currently working. SRAM doesn't save to SD card. SRAM data will only persist while the game is running.		|
| CIC Compatibility			| Complete!	 | All cic versions should work. Country version however needs to be matched with the version flashed to the board (PAL vs NTSC) |

**ROM Compatibility **
Still working towards booting at full sdk speeds (0x1240) but current v1.0.0 firmware is at 0x2040. SRAM access isn't fast enough for some titles. 

# Command Registers
| Register					| Address 	 | Length 	| Notes 				|
|--------------------------:|:-----------|:---------|----------------------:|
| Base Address				| 0x1FFE0000 | 0x1000	| 4kB buffer/Currently shared as internal buffer   			| v1/lite and v2|
| Command Interface Start	| 0x1FFE1000 | 0x0800	|						| 
| Random Seed   			| 0x1FFE0008 | 0x4		|						| 
| Start SD Read				| 0x1FFE000C | 0x4		|						| 
| SD Rom Select				| 0x1FFE0010 | 0x4		| Loads selected rom into psram	|
| SD Busy					| 0x1FFE0014 | 0x4  	| After an sd request is sent, data will be available at `Base Address` once sd busy = 0						| v2	|
| SD Read Sector Register 0	| 0x1FFE0018 | 0x4		| High bits of uint64 sector to read	|
| SD Read Sector Register 1	| 0x1FFE001C | 0x4		| Low bits of uin64 sector to read	|
| SD Read number of sectors	| 0x1FFE0020 | 0x4		| v2 code has this hard coded to always read 1 sector	|
| Select Rom				| 0x1FFE0024 | 0xFF		| Send title of rom to load from sd card e.g. "my_homebrew_rom.z64"	|

# Examples of SD card access
There is an open PR for sdfs support in the Libdragon repo! Check that out if you want to patch your libdragon installs.

Libdragon sdfs implementation for DreamDrive64. This code is just snippets and may not "just work" yet. It should give you an idea of how to use the command registers and the order that things need to be done in.
Example code to load a rom with the file's name. Updates to Libdragon's `debug.c` must be made if you plan on using file methods (e.g. f_open, f_read, etc...)
```
// From command register table above
// PC64_CIBASE_ADDRESS_START = "Command Interface start"
// PC64_REGISTER_SD_SELECT_ROM = "Select Rom"
void loadRomAtSelection(char* selectedFile) {
    char* fileToLoad = malloc(sizeof(char*) * 256);
    sprintf(fileToLoad, "%s", selectedFile);

    // Write the file name to the cart buffer
    uint32_t len_aligned32 = (strlen(fileToLoad) + 3) & (-4);
    data_cache_hit_writeback_invalidate(fileToLoad, len_aligned32);
    pi_write_raw(fileToLoad, PC64_BASE_ADDRESS_START, 0, len_aligned32);

    uint32_t sdSelectRomFilenameLength = strlen(fileToLoad);
    io_write(PC64_CIBASE_ADDRESS_START + PC64_REGISTER_SD_SELECT_ROM, sdSelectRomFilenameLength);

    wait_ms(100);
}
```

## Wait for SD
Taken from current (unmerged) libdragon implementation patch
```
static uint8_t pc64_sd_wait() {
    uint32_t timeout = 0;
    uint32_t isBusy __attribute__((aligned(8)));
	isBusy = 1;

    // Wait until the cartridge interface is ready
    do {
		
        // returns 1 while sd card is busy
		isBusy = io_read(PC64_CIBASE_ADDRESS_START + PC64_REGISTER_SD_BUSY);
        
        // Took too long, abort
        if((timeout++) > 10000000) {
			fprintf(stdout, "SD_WAIT timed out. isBusy: %ld\n", isBusy);
			return -1;
		}
    }
    while(isBusy != 0);
    (void) timeout; // Needed to stop unused variable warning

    // Success
    return 0;
}
```


## Read
Taken from current (unmerged) libdragon implementation patch
```
static DRESULT fat_disk_read_pc64(BYTE* buff, LBA_t sector, UINT count) {
	_Static_assert(FF_MIN_SS == 512, "this function assumes sector size == 512");
	_Static_assert(FF_MAX_SS == 512, "this function assumes sector size == 512");


	int retryCount = 2;
	int i = 0;
	do {
		uint64_t current_sector = sector + i;
		
		uint32_t part0 __attribute__((aligned(8)));
		part0 = (current_sector & 0xFFFFFFFF00000000LL) >> 32;

		uint32_t part1 __attribute__((aligned(8)));
		part1 = (current_sector & 0x00000000FFFFFFFFLL);

		// send sector
		io_write(PC64_CIBASE_ADDRESS_START + PC64_REGISTER_SD_READ_SECTOR0, part0);

		io_write(PC64_CIBASE_ADDRESS_START + PC64_REGISTER_SD_READ_SECTOR1, part1);

		// send num sectors to read
		io_write(PC64_CIBASE_ADDRESS_START + PC64_REGISTER_SD_READ_NUM_SECTORS, 1);

		// start the load
		io_write(PC64_CIBASE_ADDRESS_START + PC64_COMMAND_SD_READ, 1);

        // wait for the sd card to finish
        if(pc64_sd_wait() == 0) {
			data_cache_hit_writeback_invalidate(buff, 512);
			dma_read(buff, PC64_BASE_ADDRESS_START, 512);
			buff += 512;
			i++;
		} else {
			retryCount--;
			fprintf(stderr, "wait timeout\n");
		}

		if (retryCount <= 0) {
			break;
		}
		
	} while (i < count);

	return RES_OK;
}
```

## Write
Not supported yet!


# Join in on the fun!
Dreamcraft Industries discord

[Join the Dreamcraft Discord!](https://discord.gg/yawWMcv6tC)

Original PicoCart64 discord
Join the Discord server to participate in the discussion and development of this project.
[Dubious Technology Discord](https://discord.gg/2Gb3jWqqja)
