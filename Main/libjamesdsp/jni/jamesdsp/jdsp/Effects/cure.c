/* cure.c — Cure crossfeed, standalone module.
 * See cure.h for the porting notes. */

#include "cure.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Shared parameter application (clamped at the IPC trust boundary). */
static void cure_apply(cure_t *c, double sampleRate, int level)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    if (level < 0 || level > 2)
        level = 0;

    c->sampleRate = sampleRate;
    c->level = level;

    double cutoff = 700.0, g = 0.30;
    if (level == 1) {
        cutoff = 700.0;
        g = 0.40;
    }
    else if (level == 2) {
        cutoff = 650.0;
        g = 0.55;
    }
    c->lpCoef = 1.0 - exp(-2.0 * M_PI * cutoff / sampleRate);
    c->feed = g;
    c->norm = 1.0 / (1.0 + g);
}

void cure_init(cure_t *c)
{
    c->lpzL = c->lpzR = 0.0;
    cure_apply(c, 48000.0, 0);
}

void cure_configure(cure_t *c, double sampleRate, int level)
{
    cure_apply(c, sampleRate, level);
}

void cure_refresh(cure_t *c, double sampleRate)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    c->sampleRate = sampleRate;
    double cutoff = 700.0;
    if (c->level == 2)
        cutoff = 650.0;
    c->lpCoef = 1.0 - exp(-2.0 * M_PI * cutoff / sampleRate);
}

void cure_set_enabled(cure_t *c, bool enabled)
{
    if (c->enabled != enabled) {
        if (enabled)
            c->lpzL = c->lpzR = 0.0;
        c->enabled = enabled;
    }
}

void cure_process_interleaved(cure_t *c, float *data, size_t numFrames)
{
    if (!c->enabled || numFrames == 0)
        return;

    for (size_t i = 0; i < numFrames; i++) {
        const double l = data[i * 2];
        const double r = data[i * 2 + 1];
        /* The left ear gets the low-passed right channel and vice versa. */
        c->lpzL += c->lpCoef * (r - c->lpzL);
        c->lpzR += c->lpCoef * (l - c->lpzR);
        data[i * 2] = (float)((l + c->feed * c->lpzL) * c->norm);
        data[i * 2 + 1] = (float)((r + c->feed * c->lpzR) * c->norm);
    }

    if (fabs(c->lpzL) < 1e-20)
        c->lpzL = 0.0;
    if (fabs(c->lpzR) < 1e-20)
        c->lpzR = 0.0;
}
