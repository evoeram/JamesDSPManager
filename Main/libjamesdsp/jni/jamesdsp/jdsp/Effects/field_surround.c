/* field_surround.c — Field Surround, standalone module.
 * See field_surround.h for the porting notes. */

#include "field_surround.h"

#include <math.h>

void field_surround_init(field_surround_t *f)
{
    f->strengthPct = 30.0;
    f->midImagePct = 50.0;
    f->sideGain = 1.0 + 30.0 * 0.02;
    f->midGain = 0.5 + 50.0 * 0.01;
    f->enabled = false;
}

void field_surround_configure(field_surround_t *f, double strengthPct,
                              double midImagePct)
{
    /* Clamped at the IPC trust boundary; in root mode any app can send. */
    if (!isfinite(strengthPct) || strengthPct < 0.0)
        strengthPct = 0.0;
    if (strengthPct > 100.0)
        strengthPct = 100.0;
    if (!isfinite(midImagePct) || midImagePct < 0.0)
        midImagePct = 0.0;
    if (midImagePct > 100.0)
        midImagePct = 100.0;

    f->strengthPct = strengthPct;
    f->midImagePct = midImagePct;
    f->sideGain = 1.0 + strengthPct * 0.02; /* 0..100 -> 1..3 */
    f->midGain = 0.5 + midImagePct * 0.01;  /* 0..100 -> 0.5..1.5 */
}

void field_surround_set_enabled(field_surround_t *f, bool enabled)
{
    /* Stateless: the transition only toggles processing. */
    f->enabled = enabled;
}

void field_surround_process_interleaved(field_surround_t *f, float *data,
                                        size_t numFrames)
{
    if (!f->enabled || numFrames == 0)
        return;

    for (size_t i = 0; i < numFrames; i++) {
        const double l = data[i * 2];
        const double r = data[i * 2 + 1];
        const double mid = (l + r) * 0.5 * f->midGain;
        const double side = (l - r) * 0.5 * f->sideGain;
        data[i * 2] = (float)(mid + side);
        data[i * 2 + 1] = (float)(mid - side);
    }
}
