#ifndef VIPER_CLARITY_H
#define VIPER_CLARITY_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ViPER Clarity: transient sharpening (derivative boost) plus a 4.5 kHz high
 * shelf, blended by mode:
 *   0 = Natural (sharpening only)
 *   1 = Natural + shelf
 *   2 = OZone/XHiFi (shelf only)
 *
 * Ported from RootlessViPER4Android's ViperClarity (viperextras.c), reworked
 * as a standalone module like dynamicEq.c/lufs_normalizer.h. Difference from
 * the source: the shelf is redesigned on sample-rate change
 * (viper_clarity_refresh); the source keeps the rate current at SetParam. */

typedef struct {
    bool enabled;
    /* Parameters */
    double sampleRate;
    int mode;
    double gainDb;
    /* Derived coefficients (RBJ high shelf, 4.5 kHz) */
    double sharp;
    double b0, b1, b2;
    double a1, a2;
    /* State */
    double prev[2];
    double z[2][4]; /* per channel: x1, x2, y1, y2 */
} viper_clarity_t;

void viper_clarity_init(viper_clarity_t *c);
void viper_clarity_configure(viper_clarity_t *c, double sampleRate,
                             int mode, double gainDb);
void viper_clarity_refresh(viper_clarity_t *c, double sampleRate);
void viper_clarity_set_enabled(viper_clarity_t *c, bool enabled);
void viper_clarity_process_interleaved(viper_clarity_t *c, float *data,
                                       size_t numFrames);

#ifdef __cplusplus
}
#endif

#endif
