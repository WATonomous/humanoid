#include "ring_buffer.h"

void ring_buffer_init(ring_buffer_t* rb, uint8_t* buf, uint32_t size)
{
    rb->buf = buf;
    rb->size = size;
    rb->read_index = 0U;
    rb->write_index = 0U;

}

bool ring_buffer_empty(ring_buffer_t* rb)
{
    return (rb->read_index == rb->write_index);
}

static bool ring_buffer_full(ring_buffer_t* rb)
{
    return (((rb->write_index + 1U) % rb->size) == rb->read_index);
}

bool ring_buffer_read(ring_buffer_t* rb, uint8_t* byte)
{
    if (ring_buffer_empty(rb))
    {
        return false;
    }
 
    *byte = rb->buf[rb->read_index];
    rb->read_index = (rb->read_index + 1U) % rb->size;
 
    return true;
}
 
bool ring_buffer_write(ring_buffer_t* rb, uint8_t byte)
{
    if (ring_buffer_full(rb))
    {
        return false;
    }
 
    rb->buf[rb->write_index] = byte;
    rb->write_index = (rb->write_index + 1U) % rb->size;
 
    return true;
}

 


