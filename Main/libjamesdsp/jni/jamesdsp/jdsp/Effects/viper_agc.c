/* viper_agc.c — AGC, standalone module.
 * See viper_agc.h for the porting notes. */

#include "viper_agc.h"

#include <math.h>

/* Shared parameter application (clamped at the IPC trust boundary). */
static void agc_apply(viper_agc_t *a, double sampleRate,
                      double targetPct, double maxBoostDb)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    if (!isfinite(targetPct) || targetPct < 5.0)
        targetPct = 5.0;
    if (targetPct > 100.0)
        targetPct = 100.0;
    if (!isfinite(maxBoostDb) || maxBoostDb < 0.0)
        maxBoostDb = 0.0;
    if (maxBoostDb > 24.0)
        maxBoostDb = 24.0;

    a->sampleRate = sampleRate;
    a->targetPct = targetPct;
    a->maxBoostDb = maxBoostDb;
    a->target = targetPct * 0.01;
    a->maxGain = pow(10.0, maxBoostDb / 20.0);
    /* Slow envelope (~300 ms) and very slow gain glide (~1.5 s). */
    a->envCoef = 1.0 - exp(-1.0 / (0.3 * sampleRate));
    a->gainCoef = 1.0 - exp(-1.0 / (1.5 * sampleRate));
}

void viper_agc_init(viper_agc_t *a)
{
    a->env = 0.0;
    a->gain = 1.0;
    agc_apply(a, 48000.0, 30.0, 12.0);
}

void viper_agc_configure(viper_agc_t *a, double sampleRate,
                         double targetPct, double maxBoostDb)
{
    agc_apply(a, sampleRate, targetPct, maxBoostDb);
    if (a->gain < 1.0)
        a->gain = 1.0;
}

void viper_agc_refresh(viper_agc_t *a, double sampleRate)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    a->sampleRate = sampleRate;
    a->envCoef = 1.0 - exp(-1.0 / (0.3 * sampleRate));
    a->gainCoef = 1.0 - exp(-1.0 / (1.5 * sampleRate));
}

void viper_agc_set_enabled(viper_agc_t *a, bool enabled)
{
    if (a->enabled != enabled) {
        if (enabled) {
            a->env = 0.0;
            a->gain = 1.0;
        }
        a->enabled = enabled;
    }
}

void viper_agc_process_interleaved(viper_agc_t *a, float *data,
                                   size_t numFrames)
{
    if (!a->enabled || numFrames == 0)
        return;

    for (size_t i = 0; i < numFrames; i++) {
        const double l = data[i * 2];
        const double r = data[i * 2 + 1];
        double mag = fabs(l);
        const double magR = fabs(r);
        if (magR > mag)
            mag = magR;

        a->env += a->envCoef * (mag - a->env);
        if (a->env > 0.002) {
            double desired = a->target / a->env;
            if (desired > a->maxGain)
                desired = a->maxGain;
            if (desired < 0.2)
                desired = 0.2;
            a->gain += a->gainCoef * (desired - a->gain);
        }
        if (a->gain < 1e-20)
            a->gain = 0.0;

        data[i * 2] = (float)(l * a->gain);
        data[i * 2 + 1] = (float)(r * a->gain);
    }
}
