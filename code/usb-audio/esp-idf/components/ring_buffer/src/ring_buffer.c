static const char *TAG = "ring_buffer";

#include "ring_buffer.h"
#include <string.h> // for memset if needed
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" // Include this for SemaphoreHandle_t
#include "freertos/task.h"

void ring_buffer_init(ring_buffer_t *rb, float *buf, size_t size)
{
    rb->buffer = buf;
    rb->size = size;
    rb->write_index = 0;
    rb->read_index = 0;
    rb->full = false;
    rb->mutex = xSemaphoreCreateMutex();
}

void ring_buffer_reset(ring_buffer_t *rb)
{
    rb->write_index = 0;
    rb->read_index = 0;
    rb->full = false;
}

bool ring_buffer_write(ring_buffer_t *rb, float data)
{
    xSemaphoreTake(rb->mutex, portMAX_DELAY);
    if (ring_buffer_full(rb))
    {
        xSemaphoreGive(rb->mutex);
        return false;
    }
    rb->buffer[rb->write_index] = data;
    rb->write_index = (rb->write_index + 1) % rb->size;
    if (rb->write_index == rb->read_index)
    {
        rb->full = true;
    }
    xSemaphoreGive(rb->mutex);
    return true;
}

size_t ring_buffer_read_samples(ring_buffer_t *rb, float *data, size_t num_samples)
{
    size_t count = 0;

    // Lock the buffer to prevent concurrent access.
    xSemaphoreTake(rb->mutex, portMAX_DELAY);

    // Read samples until we either hit the requested amount or the buffer is empty.
    while ((count < num_samples) && !ring_buffer_empty(rb))
    {
        data[count] = rb->buffer[rb->read_index];
        rb->buffer[rb->read_index] = 0.0f; // Wipe the sample after reading it.
        rb->read_index = (rb->read_index + 1) % rb->size;
        count++;
    }

    // Once we've read samples, the buffer cannot be full.
    rb->full = false;

    // Release the lock.
    xSemaphoreGive(rb->mutex);

    return count; // Returns the number of samples actually read.
}

size_t ring_buffer_size(ring_buffer_t *rb)
{
    size_t size = rb->size;

    if (!rb->full)
    {
        if (rb->write_index >= rb->read_index)
        {
            size = rb->write_index - rb->read_index;
        }
        else
        {
            size = rb->size + rb->write_index - rb->read_index;
        }
    }
    return size;
}

size_t ring_buffer_capacity(ring_buffer_t *rb)
{
    return rb->size;
}

bool ring_buffer_empty(ring_buffer_t *rb)
{
    return (!rb->full && (rb->write_index == rb->read_index));
}

bool ring_buffer_full(ring_buffer_t *rb)
{
    return rb->full;
}
