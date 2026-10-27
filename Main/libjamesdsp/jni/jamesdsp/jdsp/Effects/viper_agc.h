#ifndef VIPER_AGC_H
#define VIPER_AGC_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* AGC (automatic gain control): slow peak-envelope follower (~300 ms) driving
 * an even slower gain glide (~1.5 s) towards a target peak level, with the
 * gain clamped to [0.2, maxBoost]. Silent passages below a floor keep the
 * last gain so noise is not amplified into a roar.
 *
 * Ported from RootlessViPER4Android's Agc (viperextras.c), reworked as a
 * standalone module like dynamicEq.c/lufs_normalizer.h. Difference from the
 * source: the envelope/gain time constants are recomputed on sample-rate
 * change (viper_agc_refresh); the source keeps the rate current at SetParam. */

typedef struct {
    bool enabled;
    /* Parameters */
    double sampleRate;
    double targetPct;  /* 0..100, mapped to a peak target 0..1 */
    double maxBoostDb; /* 0..24 */
    /* Derived coefficients */
    double target;
    double maxGain;
    double envCoef;  /* ~300 ms */
    double gainCoef; /* ~1.5 s */
    /* State */
    double env;
    double gain;
} viper_agc_t;

void viper_agc_init(viper_agc_t *a);
void viper_agc_configure(viper_agc_t *a, double sampleRate,
                         double targetPct, double maxBoostDb);
void viper_agc_refresh(viper_agc_t *a, double sampleRate);
void viper_agc_set_enabled(viper_agc_t *a, bool enabled);
void viper_agc_process_interleaved(viper_agc_t *a, float *data,
                                   size_t numFrames);

#ifdef __cplusplus
}
#endif

#endif
