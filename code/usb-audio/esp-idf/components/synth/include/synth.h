#ifndef SYNTH_H
#define SYNTH_H
#include <stdio.h>
#include "esp_err.h"

void synth_init(void);
esp_err_t synth_make_samples(int16_t *samples_in_16b, size_t q_samples);

#endif // SYNTH_H
