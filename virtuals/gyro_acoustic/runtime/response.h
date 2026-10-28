/* Quasi-steady sense mechanics/AFE followed by time-domain demodulation.
 * Equivalent to FIRE EquivalentGyroResponse with zero drive force/cross stiffness.
 * Illustrative reduced model, not calibrated hardware susceptibility.
 */
#ifndef FASTDYN_ACOUSTIC_RESPONSE_H
#define FASTDYN_ACOUSTIC_RESPONSE_H
#include <math.h>
#define ACOUSTIC_PI 3.14159265358979323846
#define ACOUSTIC_FIELDS 21

enum {
    PRESSURE, F_START, F_END, DURATION, ALTITUDE, DELAY, RAMP, PHASE,
    AREA, MASS, DRIVE_AMPLITUDE, F_REFERENCE, F_SENSE, SENSE_Q, AFE_TAU,
    DEMOD_TAU, STEP, TRACE_PERIOD, AXIS_X, AXIS_Y, AXIS_Z
};

typedef struct {
    double p[ACOUSTIC_FIELDS];
    double filter1;
    double filter2;
} AcousticResponse;

static void acoustic_transfer(const AcousticResponse *s, double frequency,
                              double *re, double *im) {
    double w = 2 * ACOUSTIC_PI * frequency;
    double ws = 2 * ACOUSTIC_PI * s->p[F_SENSE];
    double a = s->p[MASS] * (ws * ws - w * w);
    double b = s->p[MASS] * ws / s->p[SENSE_Q] * w;
    double z = w * s->p[AFE_TAU];
    double den = (a * a + b * b) * (1 + z * z);
    *re = (a - b * z) / den;
    *im = (-b - a * z) / den;
}

static double acoustic_pressure(const AcousticResponse *s, double elapsed) {
    if (elapsed < 0 || elapsed >= s->p[DURATION]) {
        return 0;
    }
    double edge = fmin(elapsed, s->p[DURATION] - elapsed);
    double envelope = s->p[RAMP] > 0 && edge < s->p[RAMP]
        ? 0.5 * (1 - cos(ACOUSTIC_PI * edge / s->p[RAMP])) : 1;
    return s->p[PRESSURE] * envelope;
}

static double acoustic_frequency(const AcousticResponse *s, double elapsed) {
    double progress = fmax(0, fmin(1, elapsed / s->p[DURATION]));
    return s->p[F_START] + (s->p[F_END] - s->p[F_START]) * progress;
}

static double acoustic_raw(const AcousticResponse *s, double elapsed) {
    double pressure = acoustic_pressure(s, elapsed);
    if (pressure == 0) {
        return 0;
    }
    double re, im, ref_re, ref_im;
    acoustic_transfer(s, acoustic_frequency(s, elapsed), &re, &im);
    acoustic_transfer(s, s->p[F_REFERENCE], &ref_re, &ref_im);
    double wr = 2 * ACOUSTIC_PI * s->p[F_REFERENCE];
    double scale = 2 * s->p[MASS] * s->p[DRIVE_AMPLITUDE] * wr * hypot(ref_re, ref_im);
    double slope = (s->p[F_END] - s->p[F_START]) / s->p[DURATION];
    /* Integral of frequency, NOT f(t)*t: retain chirp phase through resonance. */
    double delta = 2 * ACOUSTIC_PI * ((s->p[F_START] - s->p[F_REFERENCE]) * elapsed
                   + 0.5 * slope * elapsed * elapsed) + s->p[PHASE] - atan2(ref_im, ref_re);
    return -sqrt(2) * pressure * s->p[AREA] * (re * cos(delta) - im * sin(delta)) / scale;
}

static void acoustic_integrate(AcousticResponse *s, double elapsed, double dt) {
    while (dt > 1e-12) {
        double h = fmin(dt, s->p[STEP]);
        double u = acoustic_raw(s, elapsed + 0.5 * h);
        double e = exp(-h / s->p[DEMOD_TAU]);
        /* Exact two-pole update for midpoint-held input. */
        s->filter2 = u + e * (s->filter2 - u
                             + (s->filter1 - u) * h / s->p[DEMOD_TAU]);
        s->filter1 = u + (s->filter1 - u) * e;
        elapsed += h;
        dt -= h;
    }
}
#endif
