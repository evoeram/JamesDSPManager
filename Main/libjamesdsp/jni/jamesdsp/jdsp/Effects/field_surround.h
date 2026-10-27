#ifndef FIELD_SURROUND_H
#define FIELD_SURROUND_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Field Surround: mid/side reshaping — widens the side (stereo difference)
 * content and optionally attenuates or reinforces the mid image.
 *
 * Ported from RootlessViPER4Android's FieldSurround (viperextras.c), reworked
 * as a standalone module like dynamicEq.c/lufs_normalizer.h. Has no
 * sample-rate-dependent state. */

typedef struct {
    bool enabled;
    /* Parameters */
    double strengthPct; /* 0..100 */
    double midImagePct; /* 0..100 */
    /* Derived gains */
    double sideGain; /* 1 + strength*0.02  (1..3) */
    double midGain;  /* 0.5 + mid*0.01     (0.5..1.5) */
} field_surround_t;

void field_surround_init(field_surround_t *f);
void field_surround_configure(field_surround_t *f, double strengthPct,
                              double midImagePct);
void field_surround_set_enabled(field_surround_t *f, bool enabled);
void field_surround_process_interleaved(field_surround_t *f, float *data,
                                        size_t numFrames);

#ifdef __cplusplus
}
#endif

#endif
