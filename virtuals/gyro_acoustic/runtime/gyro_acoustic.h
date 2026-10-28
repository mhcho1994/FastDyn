#ifndef FASTDYN_GYRO_ACOUSTIC_H
#define FASTDYN_GYRO_ACOUSTIC_H
#include <stdbool.h>
bool gyro_acoustic_enabled(void);
typedef struct {
    double altitude_m;
    double velocity_ned[3];
    double gyro[3];
    double accel[3];
    double position_ned[3];
    double quaternion_wb[4];
    double body_rate[3];
    bool truth_valid;
} AcousticObservation;
/* Simulation/vCPU thread, after advancing the FMU. Inputs are clean sensors. */
int gyro_acoustic_step(double time_s, const AcousticObservation *observation,
                       const double pwm[4]);
/* A read does not advance time or response filters. */
void gyro_acoustic_add(double gyro[3]);
#endif
