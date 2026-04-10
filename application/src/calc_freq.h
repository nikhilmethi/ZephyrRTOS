#ifndef CALC_FREQ_H_
#define CALC_FREQ_H_

#include <stddef.h>
#include <stdint.h>

double calc_freq_zero_crossing(const int16_t *buf, size_t len, double sample_rate_hz);

#endif