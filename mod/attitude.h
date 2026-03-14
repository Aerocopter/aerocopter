#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "imu.h"

typedef enum {
    ATT_FILTER_MAHONY,
    ATT_FILTER_MADGWICK,
    ATT_FILTER_CF,
    ATT_FILTER_UKF
} att_filter_t;

void attitude_init(att_filter_t filter, float sample_freq);
void attitude_update(const imu_t *imu);
void attitude_get(float *r, float *p, float *y);

#endif
