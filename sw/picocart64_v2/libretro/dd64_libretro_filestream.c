// DreamDrive64's implementation of filestream

#include "file_stream.h"

int64_t filestream_get_size(RFILE *stream) {
    return 0;
}

RFILE* filestream_open(const char *path, unsigned mode, unsigned hints) {
    return NULL;
}

int64_t filestream_seek(RFILE *stream, int64_t offset, int seek_position) {
    return 0;
}

int64_t filestream_read(RFILE *stream, void *data, int64_t len) {
    return 0;
}

int64_t filestream_write(RFILE *stream, const void *data, int64_t len) {
    return 0;
}

int64_t filestream_tell(RFILE *stream) {
    return 0;
}

void filestream_rewind(RFILE *stream) {

}

int filestream_close(RFILE *stream) {

}

