#include "gyro_acoustic.h"
#include "response.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <fastdyn_runtime.h>

/* FMU advancement and sensor reads use the same simulation/vCPU thread. */
static AcousticResponse response;
static const VirtualContext *context;
static FILE *trace;
static bool active;
static double start_time = -1;
static double last_time;
static double next_trace;

bool gyro_acoustic_enabled(void) {
    return active;
}

void gyro_acoustic_add(double gyro[3]) {
    if (active) {
        for (int i = 0; i < 3; i++) {
            gyro[i] += response.filter2 * response.p[AXIS_X + i];
        }
    }
}

int gyro_acoustic_step(double time_s, const AcousticObservation *observation,
                       const double pwm[4]) {
    if (!active) {
        return 1;
    }
    if (time_s < last_time) {
        virtual_log(context, VIRTUAL_LOG_ERROR, "FMU time moved backwards");
        return 0;
    }
    if (start_time < 0 && observation->altitude_m >= response.p[ALTITUDE]) {
        start_time = time_s + response.p[DELAY];
        virtual_log(context, VIRTUAL_LOG_INFO,
                    "Airborne at %.3fs; sweep scheduled at %.3fs (FMU time)", time_s, start_time);
    }
    if (start_time >= 0 && time_s > start_time) {
        double from = fmax(last_time, start_time);
        acoustic_integrate(&response, from - start_time, time_s - from);
    }
    last_time = time_s;
    if (!isfinite(response.filter2)) {
        virtual_log(context, VIRTUAL_LOG_ERROR, "Nonfinite acoustic response");
        return 0;
    }
    if (time_s + 1e-9 >= next_trace) {
        const double *gyro = observation->gyro;
        const double *velocity = observation->velocity_ned;
        double elapsed = start_time >= 0 ? time_s - start_time : -1;
        double measured[3] = {gyro[0], gyro[1], gyro[2]};
        gyro_acoustic_add(measured);
        int status = fprintf(trace,
            "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
            "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
            "%.9g,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
            time_s, elapsed, acoustic_frequency(&response, elapsed),
            acoustic_pressure(&response, elapsed),
            response.filter2 * response.p[AXIS_X], response.filter2 * response.p[AXIS_Y],
            response.filter2 * response.p[AXIS_Z], gyro[0], gyro[1], gyro[2],
            measured[0], measured[1], measured[2], observation->altitude_m,
            velocity[0], velocity[1], velocity[2], pwm[0], pwm[1], pwm[2], pwm[3],
            observation->accel[0], observation->accel[1], observation->accel[2],
            observation->truth_valid, observation->position_ned[0],
            observation->position_ned[1], observation->position_ned[2],
            observation->quaternion_wb[0], observation->quaternion_wb[1],
            observation->quaternion_wb[2], observation->quaternion_wb[3],
            observation->body_rate[0], observation->body_rate[1], observation->body_rate[2]);
        if (status < 0 || fflush(trace) != 0) {
            virtual_log(context, VIRTUAL_LOG_ERROR, "Cannot write acoustic trace");
            return 0;
        }
        next_trace = time_s + response.p[TRACE_PERIOD];
    }
    return 1;
}

static void acoustic_close(void) {
    if (trace && fclose(trace) != 0) {
        virtual_log(context, VIRTUAL_LOG_ERROR, "Cannot close acoustic trace");
    }
    trace = NULL;
    active = false;
}

static int acoustic_initialize(const VirtualContext *ctx) {
    char path[4096];
    context = ctx;
    if (virtual_artifact_path(ctx, "response.txt", path, sizeof(path)) != 0) {
        return -1;
    }
    FILE *config = fopen(path, "r");
    if (!config) {
        return errno == ENOENT ? 0 : -1;
    }
    int version;
    bool valid = fscanf(config, "%d", &version) == 1 && version == 1;
    for (int i = 0; i < ACOUSTIC_FIELDS && valid; i++) {
        valid = fscanf(config, "%lf", &response.p[i]) == 1 && isfinite(response.p[i]);
        if (i != PHASE && i < AXIS_X) {
            bool zero_ok = i == PRESSURE || i == DELAY || i == RAMP;
            valid = valid && (zero_ok ? response.p[i] >= 0 : response.p[i] > 0);
        }
    }
    if (fclose(config) != 0) {
        valid = false;
    }
    double beat = fmax(fabs(response.p[F_START] - response.p[F_REFERENCE]),
                       fabs(response.p[F_END] - response.p[F_REFERENCE]));
    valid = valid && response.p[RAMP] * 2 <= response.p[DURATION]
        && response.p[STEP] >= 1e-7
        && response.p[STEP] <= fmin(response.p[DEMOD_TAU] / 20, 1 / (40 * fmax(1, beat)));
    if (!valid) {
        virtual_log(ctx, VIRTUAL_LOG_ERROR, "Invalid acoustic response artifact");
        return -1;
    }
    if (virtual_artifact_path(ctx, "trace.csv", path, sizeof(path)) != 0) {
        return -1;
    }
    trace = fopen(path, "w");
    if (!trace || fputs("time_s,sweep_elapsed_s,frequency_hz,rms_pressure_pa,"
        "false_x,false_y,false_z,truth_gyro_x,truth_gyro_y,truth_gyro_z,"
        "measured_gyro_x,measured_gyro_y,measured_gyro_z,truth_altitude_m,"
        "truth_vn,truth_ve,truth_vd,pwm1,pwm2,pwm3,pwm4,"
        "imu_accel_x,imu_accel_y,imu_accel_z,truth_valid,truth_n,truth_e,truth_d,"
        "truth_qw,truth_qx,truth_qy,truth_qz,truth_p,truth_q,truth_r\n", trace) < 0) {
        acoustic_close();
        return -1;
    }
    active = true;
    virtual_register_exit(ctx, acoustic_close);
    virtual_log(ctx, VIRTUAL_LOG_INFO, "Acoustic gyro response enabled; trace: %s", path);
    return 0;
}

VIRTUAL_PLUGIN("gyro_acoustic", acoustic_initialize);
