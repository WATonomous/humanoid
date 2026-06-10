#include "../../app/inc/common_defines.h"

#define RING_BUF_SIZE   128U

typedef struct ring_buffer_s
{
    uint32_t read_index;
    uint32_t write_index;
    uint8_t* buf;
    uint32_t size;
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t* rb, uint8_t* buf, uint32_t size)
{
    rb->buf         = buf;
    rb->size        = size;
    rb->read_index  = 0U;
    rb->write_index = 0U;

}

bool ring_buffer_read(ring_buffer_t* rb, uint8_t* byte)
{


}

bool ring_buffer_write(ring_buffer_t* rb, uint8_t byte)
{


}

bool ring_buffer_empty(ring_buffer_t* rb)
{
    return (rb->read_index == rb->write_index);
}

