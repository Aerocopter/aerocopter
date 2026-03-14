#include "attitude.h"
#include "filterCF.h"
#include "filterMahony.h"
#include "filterMadgwick.h"
#include "filterUKF.h"

static float roll, pitch, yaw;
static att_filter_t current_filter;

/* Initialize attitude solver */
void attitude_init(att_filter_t filter, float sample_freq)
{
    current_filter = filter; 
    switch(current_filter)
    {
        case ATT_FILTER_MAHONY:
	    
            Mahony_Init(sample_freq);

            break;

        case ATT_FILTER_MADGWICK:
        
            Madgwick_Init(sample_freq);

            break;

        case ATT_FILTER_CF:

            CF_Init(sample_freq);

            break;

        case ATT_FILTER_UKF:
            ;
            break;

        default:
            ;
            break;
    }
}

void attitude_update(const imu_t *imu)
{

    if (!imu) return;

    switch(current_filter)
    {
        case ATT_FILTER_MAHONY:
            
            Mahony_UpdateAHRS(
                imu->gyro[0],imu->gyro[1],imu->gyro[2],
                imu->acc[0],imu->acc[1],imu->acc[2],
                imu->mag[0],imu->mag[1],imu->mag[2]
            );
            Mahony_GetEuler(&roll, &pitch, &yaw);

            break;

	case ATT_FILTER_MADGWICK:
            
	    Madgwick_UpdateAHRS(
                imu->gyro[0],imu->gyro[1],imu->gyro[2],
                imu->acc[0],imu->acc[1],imu->acc[2],
                imu->mag[0],imu->mag[1],imu->mag[2]
            );
            Madgwick_GetEuler(&roll, &pitch, &yaw);

	    break;

        case ATT_FILTER_CF:
            
	    CF_UpdateAHRS(
                imu->gyro[0],imu->gyro[1],imu->gyro[2],
                imu->acc[0],imu->acc[1],imu->acc[2],
                imu->mag[0],imu->mag[1],imu->mag[2]
            );
            CF_GetEuler(&roll, &pitch, &yaw);

            break;

        case ATT_FILTER_UKF:
            ;
            break;

        default:
            ;
            break;
    }
}

/* Get attitude in radians */
void attitude_get(float *r, float *p, float *y)
{
    if (r) *r = roll;
    if (p) *p = pitch;
    if (y) *y = yaw;
}
