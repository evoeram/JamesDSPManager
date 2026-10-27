#ifndef FET_COMPRESSOR_H
#define FET_COMPRESSOR_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FET compressor: feed-forward peak-sensing compressor with a log-domain
 * gain computer (slope = 1 - 1/ratio) and one-pole attack/release ballistics
 * on the sidechain envelope.
 *
 * Ported from RootlessViPER4Android's FetComp (viperextras2.c), reworked as a
 * standalone module like dynamicEq.c/lufs_normalizer.h so the root effect
 * (jamesdsp.c) and the rootless wrapper share one implementation.
 *
 * Difference from the source, required by this fork's spec: attack/release
 * coefficients are recomputed on sample-rate change (fet_compressor_refresh);
 * the source keeps the rate current at SetParam time. */

typedef struct {
    bool enabled;
    /* Parameters */
    double sampleRate;
    double thresholdDb;
    double ratio;
    double attackMs;
    double releaseMs;
    double makeupDb;
    /* Derived coefficients */
    double thrLin;
    double slope;
    double attCoef;
    double relCoef;
    double makeupLin;
    /* State */
    double env;
} fet_compressor_t;

void fet_compressor_init(fet_compressor_t *f);
void fet_compressor_configure(fet_compressor_t *f, double sampleRate,
                              double thresholdDb, double ratio,
                              double attackMs, double releaseMs,
                              double makeupDb);
void fet_compressor_refresh(fet_compressor_t *f, double sampleRate);
void fet_compressor_set_enabled(fet_compressor_t *f, bool enabled);
void fet_compressor_process_interleaved(fet_compressor_t *f, float *data,
                                        size_t numFrames);

#ifdef __cplusplus
}
#endif

#endif
