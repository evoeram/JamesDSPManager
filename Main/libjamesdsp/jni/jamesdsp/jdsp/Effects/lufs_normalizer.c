/*
 * lufs_normalizer.c — LUFS loudness targeting with K-weighting and gating.
 *
 * Ported from RootlessViPER4Android's LUFSTargeting, reworked as a standalone
 * module so that the root effect (jamesdsp.c) and the rootless wrapper share
 * one implementation (same pattern as dynamicEq.c).
 *
 * Differences from the source, required by the fork's spec:
 *  - The BS.1770 relative gate (-10 LU below the absolute-gated level) is
 *    applied in a second pass over the stored window powers; the source only
 *    implements the absolute gate.
 *  - After 2 s of continuously gated-out signal (K-weighted power below the
 *    absolute gate), the measurement and the sliding-window tail are
 *    dropped and the gain eases back to 0 dB instead of sitting at the
 *    +max ceiling, so audio resuming after silence does not pump.
 *  - The K-weighting stages use the matched-biquad design of the reference
 *    BS.1770 implementations (libebur128), computed per sample rate, so any
 *    rate is measured correctly (48 kHz reproduces the published coefficient
 *    set exactly; an RBJ Q-mode shelf would bias readings by ~0.26 LU).
 */

#include "lufs_normalizer.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Canonical BS.1770 K-weighting filter design parameters (from the ITU
 * specification, same values used by reference implementations). */
#define LUFS_KW_SHELF_FREQ 1681.9744509555319
#define LUFS_KW_SHELF_GAIN_DB 3.999843853973347
#define LUFS_KW_SHELF_Q 0.7071752369554196
#define LUFS_KW_HP_FREQ 38.13547087602444
#define LUFS_KW_HP_Q 0.5003270373238773

/* LUFS = -0.691 + 10*log10(mean square K-weighted power) */
#define LUFS_OFFSET_DB (-0.691)

/* Window/step in seconds (BS.1770 momentary loudness, 75% overlap) */
#define LUFS_WINDOW_SECONDS 0.4
#define LUFS_WINDOW_OVERLAP 4

static double lufs_attack_ms(int mode)
{
    switch (mode) {
    case LUFS_MODE_SLOW: return 200.0;
    case LUFS_MODE_FAST: return 50.0;
    default: return 100.0;
    }
}

static double lufs_release_ms(int mode)
{
    switch (mode) {
    case LUFS_MODE_SLOW: return 1000.0;
    case LUFS_MODE_FAST: return 200.0;
    default: return 500.0;
    }
}

/* K-weighting stages, matched-biquad design (M. Vicanek, "Matched Biquads";
 * the formulation used by the reference BS.1770 implementation libebur128).
 * Reproduces the published ITU 48 kHz coefficient sets exactly and extends
 * to any sample rate. The shelf is a0-normalized, the high pass keeps the
 * published (1, -2, 1) numerator. */
static void lufs_build_kweighting(lufs_normalizer_t *l, double sampleRate)
{
    const double K = tan(M_PI * LUFS_KW_SHELF_FREQ / sampleRate);
    const double Vh = pow(10.0, LUFS_KW_SHELF_GAIN_DB / 20.0);
    const double Vb = pow(Vh, 0.4996667741545416);
    const double a0 = 1.0 + K / LUFS_KW_SHELF_Q + K * K;

    lufs_biquad_t *shelf[2] = { &l->kWeightL[0], &l->kWeightR[0] };
    for (int i = 0; i < 2; i++) {
        lufs_biquad_t *b = shelf[i];
        memset(b, 0, sizeof(*b));
        b->b0 = (Vh + Vb * K / LUFS_KW_SHELF_Q + K * K) / a0;
        b->b1 = 2.0 * (K * K - Vh) / a0;
        b->b2 = (Vh - Vb * K / LUFS_KW_SHELF_Q + K * K) / a0;
        b->a1 = 2.0 * (K * K - 1.0) / a0;
        b->a2 = (1.0 - K / LUFS_KW_SHELF_Q + K * K) / a0;
    }

    const double Kh = tan(M_PI * LUFS_KW_HP_FREQ / sampleRate);
    const double d = 1.0 + Kh / LUFS_KW_HP_Q + Kh * Kh;

    lufs_biquad_t *hp[2] = { &l->kWeightL[1], &l->kWeightR[1] };
    for (int i = 0; i < 2; i++) {
        lufs_biquad_t *b = hp[i];
        memset(b, 0, sizeof(*b));
        b->b0 = 1.0;
        b->b1 = -2.0;
        b->b2 = 1.0;
        b->a1 = 2.0 * (Kh * Kh - 1.0) / d;
        b->a2 = (1.0 - Kh / LUFS_KW_HP_Q + Kh * Kh) / d;
    }
}

void lufs_normalizer_init(lufs_normalizer_t *l)
{
    memset(l, 0, sizeof(*l));
    l->sampleRate = 48000.0;
    l->targetLufs = -14.0;
    l->maxGainDb = 6.0;
    l->mode = LUFS_MODE_NORMAL;
    lufs_normalizer_refresh(l, l->sampleRate);
}

void lufs_normalizer_configure(lufs_normalizer_t *l, double sampleRate,
                               double targetLufs, double maxGainDb, int mode)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    if (!isfinite(targetLufs))
        targetLufs = -14.0;
    if (!isfinite(maxGainDb) || maxGainDb < 0.0)
        maxGainDb = 6.0;
    if (mode < LUFS_MODE_SLOW || mode > LUFS_MODE_FAST)
        mode = LUFS_MODE_NORMAL;

    l->targetLufs = targetLufs;
    l->maxGainDb = maxGainDb;
    l->mode = mode;
    lufs_normalizer_refresh(l, sampleRate);
}

void lufs_normalizer_refresh(lufs_normalizer_t *l, double sampleRate)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    l->sampleRate = sampleRate;

    lufs_build_kweighting(l, sampleRate);

    l->windowSize = (size_t)(sampleRate * LUFS_WINDOW_SECONDS);
    l->stepSize = l->windowSize / LUFS_WINDOW_OVERLAP;
    if (l->stepSize == 0)
        l->stepSize = 1;
    l->silenceSampleThreshold = (size_t)(sampleRate * LUFS_SILENCE_SECONDS);

    const double attackSamples = sampleRate * lufs_attack_ms(l->mode) / 1000.0;
    const double releaseSamples = sampleRate * lufs_release_ms(l->mode) / 1000.0;
    l->attackCoef = 1.0 - exp(-1.0 / attackSamples);
    l->releaseCoef = 1.0 - exp(-1.0 / releaseSamples);

    lufs_normalizer_reset(l);
}

void lufs_normalizer_reset(lufs_normalizer_t *l)
{
    l->sampleCounter = 0;
    l->windowSampleCount = 0;
    l->windowWriteIdx = 0;
    l->windowCount = 0;
    l->windowAccumulator = 0.0;
    memset(l->windowPower, 0, sizeof(l->windowPower));
    l->measuredLufs = LUFS_ABSOLUTE_GATE_LUFS;
    l->measurementValid = false;
    l->silentSamples = 0;
    l->smoothedGainDb = 0.0;
    for (int i = 0; i < 2; i++) {
        lufs_biquad_reset(&l->kWeightL[i]);
        lufs_biquad_reset(&l->kWeightR[i]);
    }
}

void lufs_normalizer_set_enabled(lufs_normalizer_t *l, bool enabled)
{
    if (l->enabled != enabled) {
        if (enabled)
            lufs_normalizer_reset(l);
        l->enabled = enabled;
    }
}

bool lufs_normalizer_get_measured(lufs_normalizer_t *l, double *outLufs)
{
    if (!l->measurementValid)
        return false;
    if (outLufs)
        *outLufs = l->measuredLufs;
    return true;
}

/* Recompute the integrated measurement from the stored window powers.
 * Pass 1 is the absolute gate, already applied when a window was committed.
 * Pass 2 applies the relative gate: -10 LU below the absolute-gated level. */
static void lufs_recompute_measurement(lufs_normalizer_t *l)
{
    if (l->windowCount == 0) {
        l->measurementValid = false;
        l->measuredLufs = LUFS_ABSOLUTE_GATE_LUFS;
        return;
    }

    double sum = 0.0;
    for (int w = 0; w < l->windowCount; w++)
        sum += l->windowPower[w];
    const double absGatedMean = sum / (double)l->windowCount;

    if (absGatedMean <= 1e-20) {
        l->measurementValid = false;
        l->measuredLufs = LUFS_ABSOLUTE_GATE_LUFS;
        return;
    }

    const double relThreshold = absGatedMean * pow(10.0, LUFS_RELATIVE_GATE_LU / 10.0);
    double relSum = 0.0;
    int relCount = 0;
    for (int w = 0; w < l->windowCount; w++) {
        if (l->windowPower[w] > relThreshold) {
            relSum += l->windowPower[w];
            relCount++;
        }
    }

    const double gatedMean = relCount > 0 ? relSum / (double)relCount : absGatedMean;
    if (gatedMean <= 1e-20) {
        l->measurementValid = false;
        l->measuredLufs = LUFS_ABSOLUTE_GATE_LUFS;
        return;
    }

    l->measuredLufs = LUFS_OFFSET_DB + 10.0 * log10(gatedMean);
    l->measurementValid = true;
}

/* Drop the stored measurement after prolonged silence. The running window
 * accumulator is reset too: its 75%-overlap tail would otherwise keep
 * committing decaying windows above the absolute gate for seconds and
 * resurrect the measurement. smoothedGainDb is kept — the gain eases to
 * unity through the normal desired-gain path instead of jumping. */
static void lufs_clear_measurement(lufs_normalizer_t *l)
{
    l->sampleCounter = 0;
    l->windowSampleCount = 0;
    l->windowAccumulator = 0.0;
    l->windowCount = 0;
    l->windowWriteIdx = 0;
    memset(l->windowPower, 0, sizeof(l->windowPower));
    l->measuredLufs = LUFS_ABSOLUTE_GATE_LUFS;
    l->measurementValid = false;
}

/* Advance the measurement by one block. Shared by the process and the
 * measure-only paths. */
static void lufs_measure_block(lufs_normalizer_t *l, const float *data,
                               size_t numFrames)
{
    /* Absolute gate in mean-square power: LUFS = offset + 10*log10(ms)
     * crosses -70 LUFS at ms = 10^((gate - offset) / 10). */
    const double gateThreshold = pow(10.0,
        (LUFS_ABSOLUTE_GATE_LUFS - LUFS_OFFSET_DB) / 10.0);
    bool measurementDirty = false;

    for (size_t i = 0; i < numFrames; i++) {
        const double left = data[i * 2];
        const double right = data[i * 2 + 1];

        double kl = lufs_biquad_process(&l->kWeightL[0], left);
        kl = lufs_biquad_process(&l->kWeightL[1], kl);
        double kr = lufs_biquad_process(&l->kWeightR[0], right);
        kr = lufs_biquad_process(&l->kWeightR[1], kr);

        const double power = kl * kl + kr * kr;
        l->windowAccumulator += power;
        l->windowSampleCount++;
        l->sampleCounter++;

        /* Silence detector: continuous time below the absolute gate. The
         * committed windows are no use here — the sliding window's overlap
         * tail decays for seconds after loud content stops. */
        if (power < gateThreshold)
            l->silentSamples++;
        else
            l->silentSamples = 0;

        if (l->sampleCounter >= l->stepSize) {
            l->sampleCounter = 0;

            if (l->windowSampleCount >= l->windowSize) {
                const double meanSquare =
                    l->windowAccumulator / (double)l->windowSampleCount;

                if (meanSquare > gateThreshold) {
                    l->windowPower[l->windowWriteIdx] = meanSquare;
                    l->windowWriteIdx = (l->windowWriteIdx + 1) % LUFS_MAX_WINDOWS;
                    if (l->windowCount < LUFS_MAX_WINDOWS)
                        l->windowCount++;
                    measurementDirty = true;
                }

                const double shiftSamples = (double)(l->windowSize - l->stepSize);
                const double ratio = shiftSamples / (double)l->windowSampleCount;
                l->windowAccumulator *= ratio;
                l->windowSampleCount = l->windowSize - l->stepSize;
            }
        }
    }

    if (measurementDirty)
        lufs_recompute_measurement(l);

    if (l->measurementValid && l->silentSamples >= l->silenceSampleThreshold)
        lufs_clear_measurement(l);
}

void lufs_normalizer_measure_interleaved(lufs_normalizer_t *l,
                                         const float *data, size_t numFrames)
{
    lufs_measure_block(l, data, numFrames);
    for (int i = 0; i < 2; i++) {
        lufs_biquad_remove_denormals(&l->kWeightL[i]);
        lufs_biquad_remove_denormals(&l->kWeightR[i]);
    }
}

void lufs_normalizer_process_interleaved(lufs_normalizer_t *l, float *data,
                                         size_t numFrames)
{
    if (!l->enabled || numFrames == 0)
        return;

    lufs_measure_block(l, data, numFrames);

    for (size_t i = 0; i < numFrames; i++) {
        double desiredGainDb;
        if (!l->measurementValid) {
            /* No gated window yet (silence or warm-up): ease the gain back
             * to unity instead of holding the +max ceiling, so audio that
             * resumes after silence does not pump. */
            desiredGainDb = 0.0;
        }
        else {
            desiredGainDb = l->targetLufs - l->measuredLufs;
            if (desiredGainDb > l->maxGainDb)
                desiredGainDb = l->maxGainDb;
            else if (desiredGainDb < -l->maxGainDb)
                desiredGainDb = -l->maxGainDb;
        }

        const double coef = desiredGainDb > l->smoothedGainDb
                                ? l->attackCoef
                                : l->releaseCoef;
        l->smoothedGainDb += coef * (desiredGainDb - l->smoothedGainDb);

        const double gainLinear = pow(10.0, l->smoothedGainDb / 20.0);
        data[i * 2] *= (float)gainLinear;
        data[i * 2 + 1] *= (float)gainLinear;
    }

    for (int i = 0; i < 2; i++) {
        lufs_biquad_remove_denormals(&l->kWeightL[i]);
        lufs_biquad_remove_denormals(&l->kWeightR[i]);
    }
}
