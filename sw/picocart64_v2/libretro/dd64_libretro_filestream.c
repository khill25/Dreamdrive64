/**
 * @brief DreamDrive64's implementation of filestream
 * 
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Kaili Hill
 */


#include "pico/stdlib.h"
#include "file_stream.h"


int64_t filestream_get_size(RFILE *stream) {
    return f_size(stream);
}

RFILE* filestream_open(const char *path, unsigned mode, unsigned hints) {
    RFILE* fp = malloc(sizeof(FIL));
    FRESULT fr;
    fr = f_open(fp, path, FA_OPEN_EXISTING | FA_READ);

    if (FR_OK != fr && FR_EXIST != fr) {
		panic("f_open(%s) error: %s (%d)\n", path, FRESULT_str(fr), fr);
	}

    return fp;
}

int64_t filestream_seek(RFILE *stream, int64_t offset, int seek_position) {
    FRESULT res;
    switch (seek_position)
	{
	case SEEK_SET: res = f_lseek(stream, offset); break;
	case SEEK_CUR: res = f_lseek(stream, f_tell(stream) + offset); break;
	case SEEK_END: res = f_lseek(stream, f_size(stream) + offset); break;
	default: return -1;
	}
	if (res != FR_OK)
		return -1;
	return f_tell(stream);
}

int64_t filestream_read(RFILE *stream, void *data, int64_t len) {
    uint bytesRead;
    FRESULT fr = f_read(stream, data, len, &bytesRead);

    if (fr != FR_OK) {
        return -1;
    }

    // Else return the num bytes read
    return bytesRead;
}

int64_t filestream_write(RFILE *stream, const void *data, int64_t len) {
    // NOT IMPLEMENTED, this DB is read only
    return -1;
}

int64_t filestream_tell(RFILE *stream) {
    return f_tell(stream);
}

void filestream_rewind(RFILE *stream) {
    f_rewind(stream);
}

int filestream_close(RFILE *stream) {
    return f_close(stream);
}

