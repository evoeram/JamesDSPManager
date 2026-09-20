/*
 * peq_loudness.c — Pure-C PEQ biquad cascade and loudness correction.
 *
 * PEQ: RBJ Audio EQ Cookbook biquad coefficients, cascaded per-band
 *      with per-band L/R channel routing. Ported from BiQuad.h /
 *      ParametricEqProcessor.cpp.
 *
 * Loudness: Fletcher-Munson compensation with low-shelf (75 Hz) and
 *           high-shelf (10 kHz). Ported from LoudnessCorrectionProcessor.cpp
 *           (which was ported from EqualizerAPO).
 *
 * See peq_loudness.h for the public API and parameter ID documentation.
 */

#include "peq_loudness.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_LN2
#define M_LN2 0.69314718055994530942
#endif

/* ================================================================ */
/* Biquad                                                           */
/* ================================================================ */

void biquad_init(biquad_t *bq, peq_filter_type_t type, double dbGain,
                 double freq, double srate, double bandwidthOrQOrS,
                 bool isBandwidthOrS)
{
    memset(bq, 0, sizeof(*bq));

    /* Clamp frequency */
    double nyquist = srate * 0.5;
    if (freq < 1.0) freq = 1.0;
    if (freq > nyquist * 0.98) freq = nyquist * 0.98;

    double A;
    if (type == PEQ_TYPE_PEAKING || type == PEQ_TYPE_LOW_SHELF || type == PEQ_TYPE_HIGH_SHELF)
        A = pow(10.0, dbGain / 40.0);
    else
        A = pow(10.0, dbGain / 20.0);

    double omega = 2.0 * M_PI * freq / srate;
    double sn = sin(omega);
    double cs = cos(omega);
    double alpha;

    if (!isBandwidthOrS) {
        /* Q mode */
        double q = bandwidthOrQOrS;
        if (q < 0.1) q = 0.1;
        if (q > 24.0) q = 24.0;
        alpha = sn / (2.0 * q);
    } else if (type == PEQ_TYPE_LOW_SHELF || type == PEQ_TYPE_HIGH_SHELF) {
        /* S mode */
        alpha = sn / 2.0 * sqrt((A + 1.0 / A) * (1.0 / bandwidthOrQOrS - 1.0) + 2.0);
    } else {
        /* BW mode */
        alpha = sn * sinh(M_LN2 / 2.0 * bandwidthOrQOrS * omega / sn);
    }

    double beta = 2.0 * sqrt(A) * alpha;

    double b0, b1, b2, a0, a1, a2;

    switch (type) {
    case PEQ_TYPE_LOW_PASS:
        b0 = (1.0 - cs) / 2.0;
        b1 = 1.0 - cs;
        b2 = (1.0 - cs) / 2.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    case PEQ_TYPE_HIGH_PASS:
        b0 = (1.0 + cs) / 2.0;
        b1 = -(1.0 + cs);
        b2 = (1.0 + cs) / 2.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    case PEQ_TYPE_BAND_PASS:
        b0 = alpha;
        b1 = 0.0;
        b2 = -alpha;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    case PEQ_TYPE_NOTCH:
        b0 = 1.0;
        b1 = -2.0 * cs;
        b2 = 1.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    case PEQ_TYPE_ALL_PASS:
        b0 = 1.0 - alpha;
        b1 = -2.0 * cs;
        b2 = 1.0 + alpha;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cs;
        a2 = 1.0 - alpha;
        break;
    case PEQ_TYPE_PEAKING:
        b0 = 1.0 + (alpha * A);
        b1 = -2.0 * cs;
        b2 = 1.0 - (alpha * A);
        a0 = 1.0 + (alpha / A);
        a1 = -2.0 * cs;
        a2 = 1.0 - (alpha / A);
        break;
    case PEQ_TYPE_LOW_SHELF:
        b0 = A * ((A + 1.0) - (A - 1.0) * cs + beta);
        b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cs);
        b2 = A * ((A + 1.0) - (A - 1.0) * cs - beta);
        a0 = (A + 1.0) + (A - 1.0) * cs + beta;
        a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cs);
        a2 = (A + 1.0) + (A - 1.0) * cs - beta;
        break;
    case PEQ_TYPE_HIGH_SHELF:
        b0 = A * ((A + 1.0) + (A - 1.0) * cs + beta);
        b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cs);
        b2 = A * ((A + 1.0) + (A - 1.0) * cs - beta);
        a0 = (A + 1.0) - (A - 1.0) * cs + beta;
        a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cs);
        a2 = (A + 1.0) - (A - 1.0) * cs - beta;
        break;
    default:
        b0 = 1.0; b1 = 0.0; b2 = 0.0;
        a0 = 1.0; a1 = 0.0; a2 = 0.0;
        break;
    }

    /* Normalize by a0 */
    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;
}

/* ================================================================ */
/* PEQ cascade                                                      */
/* ================================================================ */

void peq_cascade_init(peq_cascade_t *peq)
{
    memset(peq, 0, sizeof(*peq));
    peq->sampleRate = 48000.0;
    peq->preampLinear = 1.0;
}

void peq_cascade_configure(peq_cascade_t *peq, double sampleRate,
                           const float *data, int numFloats)
{
    peq->sampleRate = sampleRate;
    peq->bandCount = 0;

    if (numFloats < 2)
        return;

    double preampDb = (double)data[1];
    peq->preampLinear = pow(10.0, preampDb / 20.0);

    /* Each band uses 5 floats: freq, gain, q, filterType, channelMode */
    int maxBands = (numFloats - 2) / 5;
    if (maxBands > PEQ_MAX_BANDS)
        maxBands = PEQ_MAX_BANDS;

    for (int i = 0; i < maxBands; i++) {
        int base = 2 + i * 5;
        double freq = (double)data[base + 0];
        double gain = (double)data[base + 1];
        double q    = (double)data[base + 2];
        int ftype   = (int)roundf(data[base + 3]);
        int cmode   = (int)roundf(data[base + 4]);

        /* freq <= 0 means band is disabled/unused */
        if (freq <= 0.0)
            continue;

        /* Clamp Q */
        if (q < 0.1) q = 0.1;
        if (q > 24.0) q = 24.0;

        /* Clamp filter type */
        if (ftype < 0 || ftype > 8)
            ftype = 0; /* default to peaking */

        /* Clamp channel mode */
        if (cmode < 0 || cmode > 2)
            cmode = 0;

        peq_band_t *band = &peq->bands[peq->bandCount];
        band->channelMode = cmode;
        band->enabled = true;
        band->isPreamp = false;
        band->preampLinear = 1.0;

        /* Preamp band: flat gain, no biquad */
        if (ftype == PEQ_TYPE_PREAMP) {
            band->isPreamp = true;
            band->preampLinear = pow(10.0, gain / 20.0);
            peq->bandCount++;
            continue;
        }

        /* Q mode (isBandwidthOrS = false) */
        biquad_init(&band->bqL, (peq_filter_type_t)ftype, gain, freq, sampleRate, q, false);
        biquad_init(&band->bqR, (peq_filter_type_t)ftype, gain, freq, sampleRate, q, false);

        peq->bandCount++;
    }
}

void peq_cascade_set_enabled(peq_cascade_t *peq, bool enabled)
{
    peq->enabled = enabled;
}

void peq_cascade_process_interleaved(peq_cascade_t *peq, float *data, size_t numFrames)
{
    if (!peq->enabled || peq->bandCount == 0)
        return;

    /* Apply preamp */
    if (peq->preampLinear != 1.0) {
        for (size_t i = 0; i < numFrames * 2; i++)
            data[i] = (float)(data[i] * peq->preampLinear);
    }

    /* Process through biquad cascade */
    for (size_t i = 0; i < numFrames; i++) {
        double sampleL = data[i * 2];
        double sampleR = data[i * 2 + 1];

        for (int b = 0; b < peq->bandCount; b++) {
            peq_band_t *band = &peq->bands[b];
            if (!band->enabled)
                continue;

            /* Preamp band: flat gain, no biquad processing */
            if (band->isPreamp) {
                const double g = band->preampLinear;
                switch (band->channelMode) {
                case PEQ_CHAN_BOTH:
                    sampleL *= g;
                    sampleR *= g;
                    break;
                case PEQ_CHAN_LEFT:
                    sampleL *= g;
                    break;
                case PEQ_CHAN_RIGHT:
                    sampleR *= g;
                    break;
                }
                continue;
            }

            biquad_remove_denormals(&band->bqL);
            biquad_remove_denormals(&band->bqR);

            switch (band->channelMode) {
            case PEQ_CHAN_BOTH:
                sampleL = biquad_process(&band->bqL, sampleL);
                sampleR = biquad_process(&band->bqR, sampleR);
                break;
            case PEQ_CHAN_LEFT:
                sampleL = biquad_process(&band->bqL, sampleL);
                break;
            case PEQ_CHAN_RIGHT:
                sampleR = biquad_process(&band->bqR, sampleR);
                break;
            }
        }

        data[i * 2]     = (float)sampleL;
        data[i * 2 + 1] = (float)sampleR;
    }
}

/* ================================================================ */
/* Loudness correction                                              */
/* ================================================================ */

void loudness_init(loudness_t *l)
{
    memset(l, 0, sizeof(*l));
    l->sampleRate = 48000.0;
    l->attenuation = 1.0;
    l->attFactor = 1.0;
    l->neutral = true;
    l->lastComputedVolume = 1e30; /* impossible → forces recompute */
    l->coeffsDirty = true;
}

static void loudness_get_low_shelf_params(loudness_t *l, double volume,
                                          double *freq, double *s,
                                          double *gain, double *preAmp)
{
    *freq = 75.0;
    *s = 0.52;
    double volDiff = l->referenceLevel - l->referenceOffset - volume;
    if (volDiff > 0.0) {
        *gain = volDiff * 0.55 / (1.0 - 0.55) * l->attenuation;
        *preAmp = -*gain;
    } else if (volDiff < 0.0) {
        *preAmp = 0.0;
        *gain = volDiff * 0.55 * exp(volDiff / 90.0) * l->attenuation;
    } else {
        *gain = 0.0;
        *preAmp = 0.0;
    }
}

static void loudness_get_high_shelf_params(loudness_t *l, double volume,
                                           double *freq, double *s, double *gain)
{
    *freq = 10000.0;
    *s = 0.9;
    double volDiff = l->referenceLevel - l->referenceOffset - volume;
    if (volDiff > 0.0) {
        *gain = volDiff * 0.225 * exp(-volDiff / 100.0) * l->attenuation;
    } else if (volDiff < 0.0) {
        *gain = volDiff * 0.175 * exp(volDiff / 80.0) * l->attenuation;
    } else {
        *gain = 0.0;
    }
}

static void loudness_recompute(loudness_t *l, double volume)
{
    double freqLS, sLS, gainLS, preAmp;
    double freqHS, sHS, gainHS;

    loudness_get_low_shelf_params(l, volume, &freqLS, &sLS, &gainLS, &preAmp);
    l->attFactor = exp(preAmp / 6.0 * log(2.0));

    loudness_get_high_shelf_params(l, volume + preAmp, &freqHS, &sHS, &gainHS);

    double absGainLS = fabs(gainLS);
    double absGainHS = fabs(gainHS);
    l->neutral = (absGainLS < 0.2 && absGainHS < 0.2);

    /* S-slope shelf mode (isBandwidthOrS = true) */
    biquad_init(&l->lowShelfL,  PEQ_TYPE_LOW_SHELF,  gainLS, freqLS, l->sampleRate, sLS, true);
    biquad_init(&l->lowShelfR,  PEQ_TYPE_LOW_SHELF,  gainLS, freqLS, l->sampleRate, sLS, true);
    biquad_init(&l->highShelfL, PEQ_TYPE_HIGH_SHELF, gainHS, freqHS, l->sampleRate, sHS, true);
    biquad_init(&l->highShelfR, PEQ_TYPE_HIGH_SHELF, gainHS, freqHS, l->sampleRate, sHS, true);
}

void loudness_configure(loudness_t *l, double sampleRate,
                        double refLevel, double refOffset,
                        double attenuation, double volumeDb)
{
    l->sampleRate = sampleRate;
    l->referenceLevel = refLevel;
    l->referenceOffset = refOffset;
    l->attenuation = (attenuation < 0.0) ? 0.0 : (attenuation > 2.0 ? 2.0 : attenuation);
    l->volumeDb = volumeDb;
    l->lastComputedVolume = 1e30; /* force recompute */
    l->coeffsDirty = true;
    l->attFactor = 1.0;
    l->neutral = true;
}

void loudness_set_volume(loudness_t *l, double volumeDb)
{
    l->volumeDb = volumeDb;
    l->coeffsDirty = true;
}

void loudness_set_enabled(loudness_t *l, bool enabled)
{
    l->enabled = enabled;
}

void loudness_process_interleaved(loudness_t *l, float *data, size_t numFrames)
{
    if (!l->enabled)
        return;

    /* Lazily recompute coefficients if volume or parameters changed */
    if (l->coeffsDirty) {
        loudness_recompute(l, l->volumeDb);
        l->lastComputedVolume = l->volumeDb;
        l->coeffsDirty = false;
    }

    if (l->neutral)
        return;

    for (size_t i = 0; i < numFrames; i++) {
        double sampleL = data[i * 2];
        double sampleR = data[i * 2 + 1];

        /* Low shelf → pre-amp attenuation → high shelf */
        sampleL = biquad_process(&l->lowShelfL, sampleL);
        sampleR = biquad_process(&l->lowShelfR, sampleR);

        sampleL *= l->attFactor;
        sampleR *= l->attFactor;

        sampleL = biquad_process(&l->highShelfL, sampleL);
        sampleR = biquad_process(&l->highShelfR, sampleR);

        data[i * 2]     = (float)sampleL;
        data[i * 2 + 1] = (float)sampleR;
    }

    /* Remove denormals */
    biquad_remove_denormals(&l->lowShelfL);
    biquad_remove_denormals(&l->lowShelfR);
    biquad_remove_denormals(&l->highShelfL);
    biquad_remove_denormals(&l->highShelfR);
}
