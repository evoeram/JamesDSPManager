/*
 * lufs_normalizer.h — LUFS loudness targeting with K-weighting and gating.
 *
 * Measures the integrated loudness (ITU-R BS.1770) of the playing program
 * through a sliding 400 ms window with a 100 ms step and smoothly adjusts
 * the output gain so that the measured loudness approaches a user target
 * (default -14 LUFS). The correction is clamped to a user-set maximum
 * (default ±6 dB) and smoothed with a one-pole attack/release follower.
 *
 * Pure C, no Android dependencies. The DSP is ported from
 * RootlessViPER4Android (GPL-3.0) and extended with the BS.1770 relative
 * gate (-10 LU) and silence-safe gain behaviour; see NOTICE.
 *
 * K-weighting: the two BS.1770 stages (high shelf +4 dB @ 1681.97 Hz, then
 * high-pass @ 38.14 Hz) use the matched-biquad design of the reference
 * BS.1770 implementations (libebur128), computed per sample rate. It
 * reproduces the published 48 kHz coefficient sets exactly and extends
 * correctly to arbitrary rates (an RBJ Q-mode design would bias readings
 * by ~0.26 LU around 1 kHz).
 *
 * Parameter IDs (AudioEffect API, root effect):
 *   1220 — LUFS normalizer enable/disable (short)
 *   1309 — LUFS config (3 floats = 12 bytes): [targetLufs, maxGainDb, mode]
 *          mode: 0 = slow, 1 = normal, 2 = fast gain smoothing.
 */

#ifndef LUFS_NORMALIZER_H
#define LUFS_NORMALIZER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LUFS_MAX_WINDOWS 40
#define LUFS_ABSOLUTE_GATE_LUFS (-70.0)
#define LUFS_RELATIVE_GATE_LU (-10.0)

/* Continuous time with K-weighted power below the absolute gate after which
 * the measurement is declared silent and reset, so the gain eases back to
 * unity instead of holding the last correction. */
#define LUFS_SILENCE_SECONDS 2.0

/* Smoothing presets (attack/release time constants in milliseconds) */
#define LUFS_MODE_SLOW 0
#define LUFS_MODE_NORMAL 1
#define LUFS_MODE_FAST 2

/* Private biquad state: this module must not pull in peq_loudness.h, whose
 * channel enums and macros collide with the rootless wrapper's C++ headers. */
typedef struct {
    double b0, b1, b2;
    double a1, a2;
    double x1, x2, y1, y2;
} lufs_biquad_t;

static inline double lufs_biquad_process(lufs_biquad_t *bq, double sample)
{
    const double result = bq->b0 * sample + bq->b1 * bq->x1 + bq->b2 * bq->x2
                          - bq->a1 * bq->y1 - bq->a2 * bq->y2;
    bq->x2 = bq->x1;
    bq->x1 = sample;
    bq->y2 = bq->y1;
    bq->y1 = result;
    return result;
}

static inline void lufs_biquad_reset(lufs_biquad_t *bq)
{
    bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0;
}

static inline void lufs_biquad_remove_denormals(lufs_biquad_t *bq)
{
    const double denorm_limit = 2.2250738585072014e-308;
    if (bq->x1 > -denorm_limit && bq->x1 < denorm_limit) bq->x1 = 0.0;
    if (bq->x2 > -denorm_limit && bq->x2 < denorm_limit) bq->x2 = 0.0;
    if (bq->y1 > -denorm_limit && bq->y1 < denorm_limit) bq->y1 = 0.0;
    if (bq->y2 > -denorm_limit && bq->y2 < denorm_limit) bq->y2 = 0.0;
}

typedef struct {
    bool enabled;
    /* Parameters */
    double sampleRate;
    double targetLufs;
    double maxGainDb;
    int mode; /* LUFS_MODE_SLOW / NORMAL / FAST */
    /* K-weighting chain: [0] = shelf, [1] = high-pass, per channel */
    lufs_biquad_t kWeightL[2];
    lufs_biquad_t kWeightR[2];
    /* Sliding window accumulator (BS.1770 momentary loudness) */
    size_t windowSize;
    size_t stepSize;
    size_t silenceSampleThreshold;
    size_t sampleCounter;
    size_t windowSampleCount;
    int windowWriteIdx;
    int windowCount;
    size_t silentSamples; /* consecutive samples below the absolute gate */
    double windowAccumulator;
    double windowPower[LUFS_MAX_WINDOWS];
    /* Cached integrated-loudness estimate; only recomputed when the set of
     * gated windows changes, not per sample. */
    double measuredLufs;
    bool measurementValid;
    /* Gain smoothing */
    double smoothedGainDb;
    double attackCoef;
    double releaseCoef;
} lufs_normalizer_t;

void lufs_normalizer_init(lufs_normalizer_t *l);

/* Configure parameters and rebuild all rate-dependent state. */
void lufs_normalizer_configure(lufs_normalizer_t *l, double sampleRate,
                               double targetLufs, double maxGainDb, int mode);

/* Rebuild rate-dependent state for a new sample rate, keeping the
 * parameters (root effect's SET_CONFIG handler). */
void lufs_normalizer_refresh(lufs_normalizer_t *l, double sampleRate);

void lufs_normalizer_set_enabled(lufs_normalizer_t *l, bool enabled);
void lufs_normalizer_reset(lufs_normalizer_t *l);

/* Last cached integrated-loudness estimate in LUFS. Returns false while no
 * window has passed the absolute gate (silence or measurement warming up). */
bool lufs_normalizer_get_measured(lufs_normalizer_t *l, double *outLufs);

/* Process interleaved stereo float samples in-place. */
void lufs_normalizer_process_interleaved(lufs_normalizer_t *l, float *data,
                                         size_t numFrames);

/* Measure-only variant: K-weighting + windowing + gating without applying
 * any gain. Used by the host self-test to validate BS.1770 conformance. */
void lufs_normalizer_measure_interleaved(lufs_normalizer_t *l,
                                         const float *data, size_t numFrames);

#ifdef __cplusplus
}
#endif

#endif /* LUFS_NORMALIZER_H */
