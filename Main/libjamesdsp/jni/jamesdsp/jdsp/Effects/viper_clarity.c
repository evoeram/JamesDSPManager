/* viper_clarity.c — ViPER Clarity, standalone module.
 * See viper_clarity.h for the porting notes. */

#include "viper_clarity.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CLARITY_SHELF_FREQ 4500.0

/* RBJ high shelf (Q form derived from S, as in the source's vcShelf). */
static void clarity_design_shelf(viper_clarity_t *c, double sampleRate,
                                 double gainDb)
{
    const double A = pow(10.0, gainDb / 40.0);
    const double w0 = 2.0 * M_PI * CLARITY_SHELF_FREQ / sampleRate;
    const double cw = cos(w0);
    const double sw = sin(w0);
    const double alpha = sw / 2.0 * sqrt((A + 1.0 / A) * (1.0 / 0.9 - 1.0) + 2.0);
    const double sq = 2.0 * sqrt(A) * alpha;
    const double a0 = (A + 1.0) - (A - 1.0) * cw + sq;

    c->b0 = (A * ((A + 1.0) + (A - 1.0) * cw + sq)) / a0;
    c->b1 = (-2.0 * A * ((A - 1.0) + (A + 1.0) * cw)) / a0;
    c->b2 = (A * ((A + 1.0) + (A - 1.0) * cw - sq)) / a0;
    c->a1 = (2.0 * ((A - 1.0) - (A + 1.0) * cw)) / a0;
    c->a2 = ((A + 1.0) - (A - 1.0) * cw - sq) / a0;
}

/* Shared parameter application (clamped at the IPC trust boundary). */
static void clarity_apply(viper_clarity_t *c, double sampleRate,
                          int mode, double gainDb)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    if (mode < 0 || mode > 2)
        mode = 0;
    if (!isfinite(gainDb) || gainDb < 0.0)
        gainDb = 0.0;
    if (gainDb > 14.0)
        gainDb = 14.0;

    c->sampleRate = sampleRate;
    c->mode = mode;
    c->gainDb = gainDb;
    /* Derivative sharpening strength: the transfer is 1 + sharp*(1 - z^-1),
     * which reaches 1 + 2*sharp at Nyquist, so this scale is what decides
     * whether the mode is audible at all. */
    c->sharp = gainDb * 0.10;
    clarity_design_shelf(c, sampleRate, gainDb * (mode == 2 ? 0.8 : 0.5));
}

void viper_clarity_init(viper_clarity_t *c)
{
    c->prev[0] = c->prev[1] = 0.0;
    c->z[0][0] = c->z[0][1] = c->z[0][2] = c->z[0][3] = 0.0;
    c->z[1][0] = c->z[1][1] = c->z[1][2] = c->z[1][3] = 0.0;
    clarity_apply(c, 48000.0, 0, 6.0);
}

void viper_clarity_configure(viper_clarity_t *c, double sampleRate,
                             int mode, double gainDb)
{
    clarity_apply(c, sampleRate, mode, gainDb);
}

void viper_clarity_refresh(viper_clarity_t *c, double sampleRate)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    c->sampleRate = sampleRate;
    clarity_design_shelf(c, sampleRate, c->gainDb * (c->mode == 2 ? 0.8 : 0.5));
}

void viper_clarity_set_enabled(viper_clarity_t *c, bool enabled)
{
    if (c->enabled != enabled) {
        if (enabled) {
            c->prev[0] = c->prev[1] = 0.0;
            c->z[0][0] = c->z[0][1] = c->z[0][2] = c->z[0][3] = 0.0;
            c->z[1][0] = c->z[1][1] = c->z[1][2] = c->z[1][3] = 0.0;
        }
        c->enabled = enabled;
    }
}

void viper_clarity_process_interleaved(viper_clarity_t *c, float *data,
                                       size_t numFrames)
{
    if (!c->enabled || numFrames == 0)
        return;

    const bool sharpen = c->mode == 0 || c->mode == 1;
    const bool shelf = c->mode == 1 || c->mode == 2;

    for (size_t i = 0; i < numFrames; i++) {
        for (int ch = 0; ch < 2; ch++) {
            const double x = data[i * 2 + ch];
            double y = x;
            if (sharpen) {
                /* Noise sharpening: boost sample-to-sample transitions. */
                y = x + c->sharp * (x - c->prev[ch]);
                c->prev[ch] = x;
            }
            if (shelf) {
                double *z = c->z[ch];
                const double acc = c->b0 * y + c->b1 * z[0] + c->b2 * z[1]
                                   - c->a1 * z[2] - c->a2 * z[3];
                z[1] = z[0];
                z[0] = y;
                z[3] = z[2];
                z[2] = acc;
                y = acc;
            }
            if (y < -1e20 || y > 1e20)
                y = 0.0;
            data[i * 2 + ch] = (float)y;
        }
    }
}
