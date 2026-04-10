#include "calc_freq.h"

double calc_freq_zero_crossing(const int16_t *buf, size_t len, double sample_rate_hz)
{
    size_t crossings = 0;
    size_t first_cross = 0;
    size_t last_cross = 0;
    int found_first = 0;

    if (buf == NULL || len < 3 || sample_rate_hz <= 0.0) {
        return -1.0;
    }

    for (size_t i = 1; i < len; i++) {
        if ((buf[i - 1] <= 0 && buf[i] > 0) ||
            (buf[i - 1] >= 0 && buf[i] < 0)) {
            if (!found_first) {
                first_cross = i;
                found_first = 1;
            }
            last_cross = i;
            crossings++;
        }
    }

    if (crossings < 3 || last_cross <= first_cross) {
        return -1.0;
    }

    return ((double)(crossings - 1) * sample_rate_hz) /
           (2.0 * (double)(last_cross - first_cross));
}