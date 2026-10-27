#ifndef CURE_H
#define CURE_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Cure: auditory-protection crossfeed. Each ear receives a low-passed
 * (one-pole, ~700 Hz) copy of the opposite channel, normalized by
 * 1/(1+feed) so low-frequency mono content stays at unity. This relieves
 * the extreme panning of hard-panned stereo on headphones.
 *
 * Ported from RootlessViPER4Android's Cure (viperextras2.c), reworked as a
 * standalone module like dynamicEq.c/lufs_normalizer.h. Difference from the
 * source: the low-pass coefficient is recomputed on sample-rate change
 * (cure_refresh); the source keeps the rate current at SetParam time. */

typedef struct {
    bool enabled;
    /* Parameters */
    double sampleRate;
    int level; /* 0 / 1 / 2 */
    /* Derived coefficients */
    double lpCoef;
    double feed;
    double norm;
    /* State */
    double lpzL, lpzR;
} cure_t;

void cure_init(cure_t *c);
void cure_configure(cure_t *c, double sampleRate, int level);
void cure_refresh(cure_t *c, double sampleRate);
void cure_set_enabled(cure_t *c, bool enabled);
void cure_process_interleaved(cure_t *c, float *data, size_t numFrames);

#ifdef __cplusplus
}
#endif

#endif
