/*
 * dynamicEq.h — Frequency-dependent dynamics (Dynamic EQ).
 *
 * Each band watches its own slice of the spectrum through a band-pass
 * sidechain and moves a peaking filter only while that slice is above
 * (compressor) or below (expander) its threshold. At rest the peaking
 * filter sits at 0 dB, where its coefficients are exactly the identity,
 * so an idle band does not colour the signal.
 *
 * Pure C, no Android dependencies. The DSP is ported from
 * RootlessViPER4Android (GPL-3.0); see NOTICE.
 *
 * Parameter IDs (AudioEffect API, root effect):
 *   1219 — Dynamic EQ enable/disable (short)
 *   1308 — Dynamic EQ config (float array): [mix, msMode, bandCount,
 *          band0(8 floats), band1(8 floats), ...], max 8 bands.
 *          Band layout: frequency, q, thresholdDb, ratio, attackMs,
 *          releaseMs, rangeDb, mode (0 = compressor, 1 = expander).
 *
 * m/s mode: 0 = stereo, 1 = mid, 2 = side.
 */

#ifndef DYNAMIC_EQ_H
#define DYNAMIC_EQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYNEQ_MAX_BANDS 8
#define DYNEQ_VALUES_PER_BAND 8

/* Private biquad state: this module must not pull in peq_loudness.h, whose
 * channel enums and macros collide with the rootless wrapper's C++ headers. */
typedef struct {
    double b0, b1, b2;
    double a1, a2;
    double x1, x2, y1, y2;
} dyneq_biquad_t;

static inline double dyneq_biquad_process(dyneq_biquad_t *bq, double sample)
{
    const double result = bq->b0 * sample + bq->b1 * bq->x1 + bq->b2 * bq->x2
                          - bq->a1 * bq->y1 - bq->a2 * bq->y2;
    bq->x2 = bq->x1;
    bq->x1 = sample;
    bq->y2 = bq->y1;
    bq->y1 = result;
    return result;
}

static inline void dyneq_biquad_reset(dyneq_biquad_t *bq)
{
    bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0;
}

static inline void dyneq_biquad_remove_denormals(dyneq_biquad_t *bq)
{
    const double denorm_limit = 2.2250738585072014e-308;
    if (bq->x1 > -denorm_limit && bq->x1 < denorm_limit) bq->x1 = 0.0;
    if (bq->x2 > -denorm_limit && bq->x2 < denorm_limit) bq->x2 = 0.0;
    if (bq->y1 > -denorm_limit && bq->y1 < denorm_limit) bq->y1 = 0.0;
    if (bq->y2 > -denorm_limit && bq->y2 < denorm_limit) bq->y2 = 0.0;
}

typedef enum {
    DYNEQ_MODE_COMPRESS = 0,
    DYNEQ_MODE_EXPAND = 1,
} dyneq_mode_t;

typedef enum {
    DYNEQ_MS_STEREO = 0,
    DYNEQ_MS_MID = 1,
    DYNEQ_MS_SIDE = 2,
} dyneq_ms_mode_t;

/* Per-band parameters plus the coefficients derived from them. */
typedef struct {
    double frequency;
    double q;
    double thresholdDb;
    double ratio;
    double attackMs;
    double releaseMs;
    double rangeDb;
    int mode;
    bool active;
    /* Sidechain band-pass (0 dB at the centre frequency). */
    double sb0, sb1, sb2, sa1, sa2;
    double attackCoef, releaseCoef;
} dyneq_band_cfg_t;

typedef struct {
    dyneq_band_cfg_t bands[DYNEQ_MAX_BANDS];
    int bandCount;
    double sampleRate;
    double mix;   /* 0..1 */
    int msMode;
    uint32_t version;
} dyneq_cfg_t;

/* Audio-thread state. Never touched by the configuration writer except
 * while holding cfgLock, which the audio thread also holds per block. */
typedef struct {
    dyneq_biquad_t sidechain;
    dyneq_biquad_t peak[2];
    double env;
    double gainDb;    /* target from the gain computer */
    double appliedDb; /* smoothed gain the band is moving towards */
    double coeffDb;   /* gain the peak coefficients were designed at */
} dyneq_band_state_t;

/* Double-buffered configuration, same pattern as peq_cascade_t:
 * dynamicEq_configure() builds the inactive buffer under cfgLock and then
 * flips activeIdx. The audio thread holds cfgLock for the whole block, so it
 * never reads a half-written band chain. */
typedef struct {
    dyneq_cfg_t cfg[2];
    int activeIdx;
    pthread_mutex_t cfgLock;
    bool enabled;
    /* Wet contribution 0..1. Enable/disable and a sample-rate swap both move
     * it smoothly, so the audio path never jumps between dry and wet. */
    double fade;
    double fadeCoef;
    double gainSmoothCoef;
    /* A sample-rate change is deferred until fade reaches zero; the audio
     * thread then swaps to pendingIdx and resets the filters in silence. */
    bool pendingSwap;
    int pendingIdx;
    dyneq_band_state_t state[DYNEQ_MAX_BANDS];
    uint32_t stateVersion;
    int redesignCountdown;
} dynamicEq_t;

void dynamicEq_init(dynamicEq_t *de);
void dynamicEq_destroy(dynamicEq_t *de);

/* Replace the band set. bands points to bandCount * DYNEQ_VALUES_PER_BAND
 * floats. State is preserved when only parameters change; changing the
 * sample rate resets the filters. */
void dynamicEq_configure(dynamicEq_t *de, double sampleRate,
                         const float *bands, int bandCount);

/* Update dry/wet mix (0..100 %) and M/S mode without rebuilding bands. */
void dynamicEq_set_mix_ms(dynamicEq_t *de, double mixPercent, int msMode);

/* Rebuild coefficients for a new sample rate and reset the filter state.
 * The band parameters are kept. The swap happens with the wet signal faded
 * out, so a rate change does not click. */
void dynamicEq_refresh(dynamicEq_t *de, double sampleRate);

void dynamicEq_set_enabled(dynamicEq_t *de, bool enabled);

/* True while the effect is enabled, fading out, or waiting to swap to a new
 * sample rate. Callers use this to skip the whole DSP block when nothing is
 * left to do. */
bool dynamicEq_is_active(dynamicEq_t *de);

/* Process interleaved stereo float samples. Locks cfgLock for the block. */
void dynamicEq_process_interleaved(dynamicEq_t *de, float *data, size_t numFrames);

/* Pure gain computer (no state): returns the band gain in dB.
 * env is the linear detector level; silence is floored at -120 dB. */
double dynamicEq_gain_computer_db(double env, double thresholdDb, double ratio,
                                  double rangeDb, int mode);

/* One-pole coefficient for a time constant in milliseconds. */
double dynamicEq_time_coef(double ms, double sampleRate);

/* One-pole envelope step: attack when the magnitude rises, release when it
 * falls. */
double dynamicEq_env_step(double env, double magnitude,
                          double attackCoef, double releaseCoef);

#ifdef __cplusplus
}
#endif

#endif /* DYNAMIC_EQ_H */
