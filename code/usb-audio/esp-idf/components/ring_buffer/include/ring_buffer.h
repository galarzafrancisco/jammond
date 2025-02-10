#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" // Include this for SemaphoreHandle_t
#include "freertos/task.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // Ring buffer structure
    typedef struct
    {
        float *buffer;      // Pointer to the actual data buffer
        size_t size;        // Total size of the buffer (capacity)
        size_t write_index; // Index of the next write
        size_t read_index;  // Index of the next read
        bool full;          // Flag indicating if the buffer is full
        SemaphoreHandle_t mutex;
    } ring_buffer_t;

    /**
     * @brief Initialize a ring buffer with an external memory buffer.
     *
     * @param rb    Pointer to the ring buffer handle
     * @param buf   Pointer to memory that will hold the ring buffer data
     * @param size  Size of the provided buffer in bytes
     */
    void ring_buffer_init(ring_buffer_t *rb, float *buf, size_t size);

    /**
     * @brief Reset the ring buffer to an empty state.
     *
     * @param rb  Pointer to the ring buffer
     */
    void ring_buffer_reset(ring_buffer_t *rb);

    /**
     * @brief Write a byte to the ring buffer.
     *
     * @param rb    Pointer to the ring buffer
     * @param data  The byte to write
     * @return true if successful, false if the buffer was full
     */
    bool ring_buffer_write(ring_buffer_t *rb, float data);

    /**
     * @brief Read a byte from the ring buffer.
     *
     * @param rb    Pointer to the ring buffer
     * @param data  Pointer to where the read byte should be stored
     * @param num_samples  Number of samples to read
     * @return number of samples read
     */
    size_t ring_buffer_read_samples(ring_buffer_t *rb, float *data, size_t num_samples);

    /**
     * @brief Get the number of bytes currently stored in the buffer.
     *
     * @param rb  Pointer to the ring buffer
     * @return Number of bytes in the buffer
     */
    size_t ring_buffer_size(ring_buffer_t *rb);

    /**
     * @brief Get the total capacity of the ring buffer.
     *
     * @param rb  Pointer to the ring buffer
     * @return Capacity of the buffer
     */
    size_t ring_buffer_capacity(ring_buffer_t *rb);

    /**
     * @brief Check if the ring buffer is empty.
     *
     * @param rb  Pointer to the ring buffer
     * @return true if empty, false otherwise
     */
    bool ring_buffer_empty(ring_buffer_t *rb);

    /**
     * @brief Check if the ring buffer is full.
     *
     * @param rb  Pointer to the ring buffer
     * @return true if full, false otherwise
     */
    bool ring_buffer_full(ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif // RING_BUFFER_H
