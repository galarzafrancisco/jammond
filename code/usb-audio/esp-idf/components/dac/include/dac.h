#ifndef DAC_H
#define DAC_H
#include <stdio.h>
#include "esp_err.h"

void dac_init(void);
esp_err_t dac_write(int16_t *samples_in_16b, size_t q_samples, size_t *q_samples_written);

#endif // DAC_H
