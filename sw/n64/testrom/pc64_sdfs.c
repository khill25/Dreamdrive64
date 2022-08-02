/*

This code needs to live in libdragon and will also need to modify the debug.c file to include definitions for picocart64
to use functions in here


*/
// /**
//  * SPDX-License-Identifier: BSD-2-Clause
//  *
//  * Copyright (c) 2022 Kaili Hill
//  * 
//  */

// #include <libdragon.h>
// #include "pc64_regs.h"
// #include "pc64_utils.h"
// #include "pc64_sdfs.h"

// uint8_t pc64_sd_wait() {
//     u32 timeout = 0;
//     uint32_t *read_buf32 = (uint32_t *) read_buf;
    
//     // Wait until the cartridge interface is ready
//     do {
//         // returns 1 while sd card is busy
//         pi_read_raw(read_buf, PC64_CIBASE_ADDRESS_START, PC64_REGISTER_SD_BUSY, sizeof(uint32_t))
        
//         // Took too long, abort
//         if((timeout++) > 10000)
//             return -1;
//     }
//     while(read_buf[0]);
//     (void) timeout; // Needed to stop unused variable warning
    
//     // Success
//     return 0;
// }

// DRESULT fat_disk_read_pc64(BYTE* buff, LBA_t sector, UINT count) {
// 	_Static_assert(FF_MIN_SS == 512, "this function assumes sector size == 512");
// 	_Static_assert(FF_MAX_SS == 512, "this function assumes sector size == 512");

// 	for (int i=0;i<count;i++) {
// 		// pc64_sd_wait();
// 		io_write(PC64_CIBASE_ADDRESS_START + PC64_REGISTER_SD_READ_SECTOR, sector+i);
//         io_write(PC64_CIBASE_ADDRESS_START + PC64_REGISTER_SD_READ_NUM_SECTORS, count);
// 		// pc64_sd_wait();
// 		io_write(PC64_CIBASE_ADDRESS_START + PC64_REGISTER_SD_READ, PC64_COMMAND_SD_READ);

//         // wait for the sd card to finish
//         pc64_sd_wait();

//         // TODO add in error handling and associated registers
// 		// if (pc64_sd_wait() != 0)
// 		// {
// 		// 	debugf("[debug] fat_disk_read_64drive: wait timeout\n");
// 		// 	// Operation is taking too long. Probably SD was not inserted.
// 		// 	// Send a COMMAND_ABORT and SD_RESET, and return I/O error.
// 		// 	// Note that because of a 64drive firmware bug, this is not
// 		// 	// sufficient to unblock the 64drive. The USB channel will stay
// 		// 	// unresponsive. We don't currently have a workaround for this.
// 		// 	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_COMMAND, D64_COMMAND_ABORT);
// 		// 	usb_64drive_wait();
// 		// 	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_COMMAND, D64_COMMAND_SD_RESET);
// 		// 	usb_64drive_wait();
// 		// 	return FR_DISK_ERR;
// 		// }

// 		data_cache_hit_writeback_invalidate(buff, 512);
// 		dma_read(buff, PC64_BASE_ADDRESS_START, 512);
// 		buff += 512;
// 	}
// 	return RES_OK;
// }

// // static DRESULT fat_disk_write_64drive(const BYTE* buff, LBA_t sector, UINT count)
// // {
// // 	_Static_assert(FF_MIN_SS == 512, "this function assumes sector size == 512");
// // 	_Static_assert(FF_MAX_SS == 512, "this function assumes sector size == 512");

// // 	for (int i=0;i<count;i++)
// // 	{
// // 		if (((uint32_t)buff & 7) == 0)
// // 		{
// // 			data_cache_hit_writeback(buff, 512);
// // 			dma_write(buff, D64_CIBASE_ADDRESS + D64_BUFFER, 512);
// // 		}
// // 		else
// // 		{
// // 			typedef uint32_t u_uint32_t __attribute__((aligned(1)));

// // 			uint32_t* dst = (uint32_t*)(D64_CIBASE_ADDRESS + D64_BUFFER);
// // 			u_uint32_t* src = (u_uint32_t*)buff;
// // 			for (int i = 0; i < 512/16; i++)
// // 			{
// // 				uint32_t a = *src++; uint32_t b = *src++; uint32_t c = *src++; uint32_t d = *src++;
// // 				*dst++ = a;          *dst++ = b;          *dst++ = c;          *dst++ = d;
// // 			}
// // 		}

// // 		usb_64drive_wait();
// // 		io_write(D64_CIBASE_ADDRESS + D64_REGISTER_LBA, sector+i);
// // 		usb_64drive_wait();
// // 		io_write(D64_CIBASE_ADDRESS + D64_REGISTER_COMMAND, D64_COMMAND_SD_WRITE);
// // 		if (usb_64drive_wait() != 0)
// // 		{
// // 			debugf("[debug] fat_disk_write_64drive: wait timeout\n");
// // 			// Operation is taking too long. Probably SD was not inserted.
// // 			// Send a COMMAND_ABORT and SD_RESET, and return I/O error.
// // 			// Note that because of a 64drive firmware bug, this is not
// // 			// sufficient to unblock the 64drive. The USB channel will stay
// // 			// unresponsive. We don't currently have a workaround for this.
// // 			io_write(D64_CIBASE_ADDRESS + D64_REGISTER_COMMAND, D64_COMMAND_ABORT);
// // 			usb_64drive_wait();
// // 			io_write(D64_CIBASE_ADDRESS + D64_REGISTER_COMMAND, D64_COMMAND_SD_RESET);
// // 			usb_64drive_wait();
// // 			return FR_DISK_ERR;
// // 		}

// // 		buff += 512;
// // 	}

// // 	return RES_OK;
// // }

// static DSTATUS fat_disk_initialize_pc64(void) { return 0; }