/* fet_compressor.c — FET compressor, standalone module.
 * See fet_compressor.h for the porting notes. */

#include "fet_compressor.h"

#include <math.h>

static double fet_attack_coef(double sampleRate, double attackMs)
{
    double samples = sampleRate * attackMs * 0.001;
    if (samples < 1.0)
        samples = 1.0;
    return 1.0 - exp(-1.0 / samples);
}

static double fet_release_coef(double sampleRate, double releaseMs)
{
    double samples = sampleRate * releaseMs * 0.001;
    if (samples < 1.0)
        samples = 1.0;
    return 1.0 - exp(-1.0 / samples);
}

/* Shared coefficient computation; parameters are clamped at the IPC trust
 * boundary (in root mode any app can send them) so that absurd values cannot
 * produce inf/NaN gains. */
static void fet_apply(fet_compressor_t *f, double sampleRate,
                      double thresholdDb, double ratio,
                      double attackMs, double releaseMs, double makeupDb)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    if (!isfinite(thresholdDb) || thresholdDb < -120.0)
        thresholdDb = -120.0;
    if (thresholdDb > 24.0)
        thresholdDb = 24.0;
    if (!isfinite(ratio) || ratio < 1.0)
        ratio = 1.0;
    if (ratio > 100.0)
        ratio = 100.0;
    if (!isfinite(attackMs) || attackMs < 0.05)
        attackMs = 0.05;
    if (attackMs > 5000.0)
        attackMs = 5000.0;
    if (!isfinite(releaseMs) || releaseMs < 5.0)
        releaseMs = 5.0;
    if (releaseMs > 60000.0)
        releaseMs = 60000.0;
    if (!isfinite(makeupDb) || makeupDb < -24.0)
        makeupDb = -24.0;
    if (makeupDb > 24.0)
        makeupDb = 24.0;

    f->sampleRate = sampleRate;
    f->thresholdDb = thresholdDb;
    f->ratio = ratio;
    f->attackMs = attackMs;
    f->releaseMs = releaseMs;
    f->makeupDb = makeupDb;
    f->thrLin = pow(10.0, thresholdDb / 20.0);
    f->slope = 1.0 - 1.0 / ratio;
    f->attCoef = fet_attack_coef(sampleRate, attackMs);
    f->relCoef = fet_release_coef(sampleRate, releaseMs);
    f->makeupLin = pow(10.0, makeupDb / 20.0);
}

void fet_compressor_init(fet_compressor_t *f)
{
    f->env = 0.0;
    fet_apply(f, 48000.0, -18.0, 4.0, 5.0, 120.0, 0.0);
}

void fet_compressor_configure(fet_compressor_t *f, double sampleRate,
                              double thresholdDb, double ratio,
                              double attackMs, double releaseMs,
                              double makeupDb)
{
    fet_apply(f, sampleRate, thresholdDb, ratio, attackMs, releaseMs, makeupDb);
}

void fet_compressor_refresh(fet_compressor_t *f, double sampleRate)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    f->sampleRate = sampleRate;
    f->attCoef = fet_attack_coef(sampleRate, f->attackMs);
    f->relCoef = fet_release_coef(sampleRate, f->releaseMs);
}

void fet_compressor_set_enabled(fet_compressor_t *f, bool enabled)
{
    if (f->enabled != enabled) {
        if (enabled)
            f->env = 0.0;
        f->enabled = enabled;
    }
}

void fet_compressor_process_interleaved(fet_compressor_t *f, float *data,
                                        size_t numFrames)
{
    if (!f->enabled || numFrames == 0)
        return;

    for (size_t i = 0; i < numFrames; i++) {
        const double l = data[i * 2];
        const double r = data[i * 2 + 1];
        double mag = fabs(l);
        const double magR = fabs(r);
        if (magR > mag)
            mag = magR;

        if (mag > f->env)
            f->env += f->attCoef * (mag - f->env);
        else
            f->env += f->relCoef * (mag - f->env);
        if (f->env < 1e-20)
            f->env = 0.0;

        double gain = 1.0;
        if (f->env > f->thrLin && f->env > 1e-8) {
            const double overDb = 20.0 * log10(f->env / f->thrLin);
            gain = pow(10.0, -overDb * f->slope / 20.0);
        }
        gain *= f->makeupLin;

        data[i * 2] = (float)(l * gain);
        data[i * 2 + 1] = (float)(r * gain);
    }
}
