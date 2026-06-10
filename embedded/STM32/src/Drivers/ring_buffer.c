#include "../../app/inc/common_defines.h"

#define RING_BUF_SIZE 128U

typedef struct ring_buffer_s {
  uint32_t read_index;
  uint32_t write_index;
  uint8_t* buf;
  uint32_t size;
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t* rb, uint8_t* buf, uint32_t size) {}

bool ring_buffer_read(ring_buffer_t* rb, uint8_t* byte) {
  if (ring_buffer_empty(rb)) {
    return false;
  }
  *byte = rb->buf[rb->read_index];
  rb->read_index = (rb->read_index + 1) & (rb->size - 1);
  return true;
}

bool ring_buffer_write(ring_buffer_t* rb, uint8_t byte) {
  uint32_t next_write_index = (rb->write_index + 1) & (rb->size - 1);
  if (next_write_index == rb->read_index) {
    return false;
  }
  rb->buf[rb->write_index] = byte;
  rb->write_index = next_write_index;
  return true;
}

bool ring_buffer_empty(ring_buffer_t* rb) {}
