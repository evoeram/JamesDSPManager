/*
 * dynamicEq.c — Frequency-dependent dynamics (Dynamic EQ).
 *
 * Ported from RootlessViPER4Android's DynamicEq, reworked as a standalone
 * module with a double-buffered configuration and private biquad state so that the
 * root effect (jamesdsp.c) and the rootless wrapper share one implementation.
 *
 * A static cut is always there, so taming a boom that happens on four notes
 * costs body on everything else. Each band here watches its own slice of the
 * spectrum through a band-pass sidechain, and moves a peaking filter only
 * while that slice is over (or under) a threshold. At rest the filter sits at
 * 0 dB, where a peaking biquad is exactly the identity, so an idle band is
 * transparent rather than merely quiet.
 *
 * The sidechain is the mono sum. Detecting per channel would let the two
 * sides duck by different amounts, which moves the stereo image on every
 * transient.
 *
 * Coefficients are redesigned on a schedule rather than per sample: a peaking
 * biquad costs a handful of transcendentals to design and the gain cannot
 * move meaningfully within a few samples anyway.
 */

#include "dynamicEq.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* How often the peaking coefficients are redesigned, in samples. 32 is
 * 0.67 ms at 48 kHz - far shorter than the fastest useful attack. */
#define DYNEQ_REDESIGN 32
/* Below this the gain is treated as no change at all, which keeps an idle
 * band on the identity path. */
#define DYNEQ_EPS_DB 0.02
/* Detector floor: silence must not send the log to -inf and an expander band
 * to full boost on a digital black passage. */
#define DYNEQ_ENV_FLOOR 1e-6
/* Gain smoothing. The envelope is already continuous; this keeps the audible
 * gain continuous too when a threshold, ratio or range changes under a
 * running signal, and it removes the 32-sample redesign staircase. */
#define DYNEQ_GAIN_SMOOTH_MS 2.0
/* Wet fade for enable/disable and for the deferred sample-rate swap. */
#define DYNEQ_FADE_MS 5.0
#define DYNEQ_FADE_SNAP 1e-3

static double dynEqClamp(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double dynamicEq_time_coef(double ms, double sampleRate)
{
    if (ms <= 0.0 || sampleRate <= 0.0)
        return 0.0;
    return exp(-1.0 / ((ms * 0.001) * sampleRate));
}

/* One-pole step fraction for a time constant, i.e. 1 - decay, so that a
 * sample loop can do `x += (target - x) * coef`. */
static double dynEqSmoothStep(double ms, double sampleRate)
{
    return 1.0 - dynamicEq_time_coef(ms, sampleRate);
}

double dynamicEq_env_step(double env, double magnitude,
                          double attackCoef, double releaseCoef)
{
    const double c = magnitude > env ? attackCoef : releaseCoef;
    return magnitude + c * (env - magnitude);
}

double dynamicEq_gain_computer_db(double env, double thresholdDb, double ratio,
                                  double rangeDb, int mode)
{
    if (ratio < 1.0)
        ratio = 1.0;
    if (rangeDb == 0.0)
        return 0.0;

    const double level = env > DYNEQ_ENV_FLOOR ? env : DYNEQ_ENV_FLOOR;
    /* 20*log10(x) == 6.020599913279624 * log2(x); log2 is the cheaper call. */
    const double envDb = 6.020599913279624 * log2(level);
    const double over = envDb - thresholdDb;
    double target = 0.0;

    if (mode == DYNEQ_MODE_COMPRESS) {
        /* Cut once it climbs past the threshold. */
        if (over > 0.0)
            target = -over * (1.0 - 1.0 / ratio);
    } else {
        /* Lift what falls below it. */
        if (over < 0.0)
            target = -over * (1.0 - 1.0 / ratio);
    }

    /* rangeDb carries the sign the user asked for, so clamp toward it rather
     * than assuming a direction. */
    if (rangeDb < 0.0)
        return dynEqClamp(target, rangeDb, 0.0);
    return dynEqClamp(target, 0.0, rangeDb);
}

/* Sidechain band-pass, constant 0 dB at the centre. The peak-gain form
 * rather than the constant-skirt one: the threshold is meant to mean "this
 * band reached this level", which only holds if a tone at the centre
 * frequency passes through at unity. */
static void dynEqDesignSidechain(dyneq_band_cfg_t *b, double fs)
{
    const double w0 = 2.0 * M_PI * b->frequency / fs;
    const double alpha = sin(w0) / (2.0 * b->q);
    const double a0 = 1.0 + alpha;
    b->sb0 = alpha / a0;
    b->sb1 = 0.0;
    b->sb2 = -alpha / a0;
    b->sa1 = (-2.0 * cos(w0)) / a0;
    b->sa2 = (1.0 - alpha) / a0;
}

/* The band the listener hears: an RBJ peaking filter at the current gain.
 * At gain 0 the coefficients match a term for term. */
static void dynEqDesignPeaking(double frequency, double q, double fs,
                               double gainDb,
                               double *b0, double *b1, double *b2,
                               double *a1, double *a2)
{
    const double A = pow(10.0, gainDb / 40.0);
    const double w0 = 2.0 * M_PI * frequency / fs;
    const double alpha = sin(w0) / (2.0 * q);
    const double a0 = 1.0 + alpha / A;
    *b0 = (1.0 + alpha * A) / a0;
    *b1 = (-2.0 * cos(w0)) / a0;
    *b2 = (1.0 - alpha * A) / a0;
    *a1 = (-2.0 * cos(w0)) / a0;
    *a2 = (1.0 - alpha / A) / a0;
}

static void dynEqSetPeakCoeffs(dyneq_band_state_t *st,
                               double b0, double b1, double b2,
                               double a1, double a2)
{
    st->peak[0].b0 = b0;
    st->peak[0].b1 = b1;
    st->peak[0].b2 = b2;
    st->peak[0].a1 = a1;
    st->peak[0].a2 = a2;
    st->peak[1].b0 = b0;
    st->peak[1].b1 = b1;
    st->peak[1].b2 = b2;
    st->peak[1].a1 = a1;
    st->peak[1].a2 = a2;
}

static void dynEqSetIdentityPeak(dyneq_band_state_t *st)
{
    dynEqSetPeakCoeffs(st, 1.0, 0.0, 0.0, 0.0, 0.0);
}

static void dynEqResetState(dyneq_band_state_t *st)
{
    dyneq_biquad_reset(&st->sidechain);
    dyneq_biquad_reset(&st->peak[0]);
    dyneq_biquad_reset(&st->peak[1]);
    dynEqSetIdentityPeak(st);
    st->env = 0.0;
    st->gainDb = 0.0;
    st->appliedDb = 0.0;
    st->coeffDb = 0.0;
}

static void dynEqResetAll(dynamicEq_t *de)
{
    for (int i = 0; i < DYNEQ_MAX_BANDS; i++)
        dynEqResetState(&de->state[i]);
    de->redesignCountdown = 0;
}

/* Copy the active configuration into the runtime state. Sidechain
 * coefficients follow the config; the peaking filter is redesigned at the
 * gain currently applied, so a band that happens to be ducking when the
 * parameters change keeps ducking by the same amount. */
static void dynEqApplyConfigLocked(dynamicEq_t *de, const dyneq_cfg_t *cfg)
{
    const double fs = cfg->sampleRate > 0.0 ? cfg->sampleRate : 48000.0;
    de->gainSmoothCoef = dynEqSmoothStep(DYNEQ_GAIN_SMOOTH_MS, fs);
    de->fadeCoef = dynEqSmoothStep(DYNEQ_FADE_MS, fs);
    for (int i = 0; i < cfg->bandCount; i++) {
        const dyneq_band_cfg_t *b = &cfg->bands[i];
        dyneq_band_state_t *st = &de->state[i];
        double b0, b1, b2, a1, a2;

        st->sidechain.b0 = b->sb0;
        st->sidechain.b1 = b->sb1;
        st->sidechain.b2 = b->sb2;
        st->sidechain.a1 = b->sa1;
        st->sidechain.a2 = b->sa2;

        /* Keep the audible gain where it is while the filters move to the
         * new band parameters. A zero-gain band gets the exact identity
         * coefficients rather than the gain-0 peaking design, so its output
         * is bit-identical to the input. */
        if (st->coeffDb == 0.0)
        {
            dynEqSetIdentityPeak(st);
        }
        else
        {
            dynEqDesignPeaking(b->frequency, b->q, fs, st->coeffDb,
                               &b0, &b1, &b2, &a1, &a2);
            dynEqSetPeakCoeffs(st, b0, b1, b2, a1, a2);
        }
    }
    de->stateVersion = cfg->version;
}

static void dynEqFillBand(dyneq_band_cfg_t *b, const float *v, double fs)
{
    b->frequency = dynEqClamp((double)v[0], 20.0, fs * 0.45);
    b->q = dynEqClamp((double)v[1], 0.05, 24.0);
    b->thresholdDb = dynEqClamp((double)v[2], -120.0, 0.0);
    b->ratio = dynEqClamp((double)v[3], 1.0, 100.0);
    b->attackMs = dynEqClamp((double)v[4], 0.0, 5000.0);
    b->releaseMs = dynEqClamp((double)v[5], 0.0, 5000.0);
    b->rangeDb = dynEqClamp((double)v[6], -48.0, 48.0);
    b->mode = (int)v[7];
    if (b->mode != DYNEQ_MODE_EXPAND)
        b->mode = DYNEQ_MODE_COMPRESS;

    b->active = b->rangeDb != 0.0 && b->ratio > 1.0;
    b->attackCoef = dynamicEq_time_coef(b->attackMs, fs);
    b->releaseCoef = dynamicEq_time_coef(b->releaseMs, fs);
    dynEqDesignSidechain(b, fs);
}

void dynamicEq_init(dynamicEq_t *de)
{
    memset(de, 0, sizeof(*de));
    pthread_mutex_init(&de->cfgLock, NULL);
    de->cfg[0].mix = 1.0;
    de->cfg[1].mix = 1.0;
    de->cfg[0].sampleRate = 48000.0;
    de->cfg[1].sampleRate = 48000.0;
    de->stateVersion = 0;
    de->fade = 0.0;
    de->fadeCoef = dynEqSmoothStep(DYNEQ_FADE_MS, 48000.0);
    de->gainSmoothCoef = dynEqSmoothStep(DYNEQ_GAIN_SMOOTH_MS, 48000.0);
    dynEqResetAll(de);
}

void dynamicEq_destroy(dynamicEq_t *de)
{
    pthread_mutex_destroy(&de->cfgLock);
}

void dynamicEq_configure(dynamicEq_t *de, double sampleRate,
                         const float *bands, int bandCount)
{
    pthread_mutex_lock(&de->cfgLock);

    dyneq_cfg_t *active = &de->cfg[de->activeIdx];
    dyneq_cfg_t *next = &de->cfg[1 - de->activeIdx];
    const double fs = sampleRate > 0.0 ? sampleRate : 48000.0;
    const bool rateChanged = active->sampleRate > 0.0 &&
                             fabs(fs - active->sampleRate) > 0.5;
    const int prevCount = active->bandCount;

    if (bandCount < 0) bandCount = 0;
    if (bandCount > DYNEQ_MAX_BANDS) bandCount = DYNEQ_MAX_BANDS;

    memset(next, 0, sizeof(*next));
    next->sampleRate = fs;
    next->mix = active->mix;
    next->msMode = active->msMode;
    next->bandCount = bandCount;
    next->version = active->version + 1;

    if (bands != NULL) {
        for (int i = 0; i < bandCount; i++)
            dynEqFillBand(&next->bands[i], bands + i * DYNEQ_VALUES_PER_BAND, fs);
    }

    /* A different band layout or a moved/resized band changes the filter
     * response itself, which gain smoothing cannot hide. Swap those with a
     * short fade as well. Threshold, ratio and range changes stay on the
     * direct path and are absorbed by the gain smoother. */
    bool structuralChange = bandCount != prevCount;
    for (int i = 0; !structuralChange && i < bandCount; i++)
    {
        if (fabs(next->bands[i].frequency - active->bands[i].frequency) > 0.01 ||
            fabs(next->bands[i].q - active->bands[i].q) > 1e-4 ||
            next->bands[i].active != active->bands[i].active)
            structuralChange = true;
    }

    if (rateChanged || structuralChange)
    {
        /* The new configuration is applied once the wet signal has faded
         * out; resetting the filters there cannot produce a step. */
        de->pendingSwap = true;
        de->pendingIdx = 1 - de->activeIdx;
        if (de->fade <= 0.0)
        {
            de->activeIdx = de->pendingIdx;
            de->pendingSwap = false;
            dynEqResetAll(de);
            dynEqApplyConfigLocked(de, &de->cfg[de->activeIdx]);
        }
    }
    else
    {
        de->pendingSwap = false;
        /* Slots that did not exist in the previous configuration may still
         * hold another band's detector and gain; start them at rest so a
         * freshly added band does not jump to a stale ducking amount. */
        for (int i = prevCount; i < bandCount; i++)
            dynEqResetState(&de->state[i]);

        de->activeIdx = 1 - de->activeIdx;
        dynEqApplyConfigLocked(de, &de->cfg[de->activeIdx]);
    }

    pthread_mutex_unlock(&de->cfgLock);
}

void dynamicEq_set_mix_ms(dynamicEq_t *de, double mixPercent, int msMode)
{
    pthread_mutex_lock(&de->cfgLock);
    const double mix = dynEqClamp(mixPercent * 0.01, 0.0, 1.0);
    if (msMode < 0 || msMode > DYNEQ_MS_SIDE)
        msMode = DYNEQ_MS_STEREO;
    de->cfg[0].mix = mix;
    de->cfg[0].msMode = msMode;
    de->cfg[1].mix = mix;
    de->cfg[1].msMode = msMode;
    pthread_mutex_unlock(&de->cfgLock);
}

void dynamicEq_refresh(dynamicEq_t *de, double sampleRate)
{
    pthread_mutex_lock(&de->cfgLock);

    dyneq_cfg_t *active = &de->cfg[de->activeIdx];
    dyneq_cfg_t *next = &de->cfg[1 - de->activeIdx];
    const double fs = sampleRate > 0.0 ? sampleRate : 48000.0;

    *next = *active;
    next->sampleRate = fs;
    next->version = active->version + 1;
    for (int i = 0; i < next->bandCount; i++) {
        dyneq_band_cfg_t *b = &next->bands[i];
        b->frequency = dynEqClamp(b->frequency, 20.0, fs * 0.45);
        b->attackCoef = dynamicEq_time_coef(b->attackMs, fs);
        b->releaseCoef = dynamicEq_time_coef(b->releaseMs, fs);
        dynEqDesignSidechain(b, fs);
    }

    if (de->fade <= 0.0)
    {
        de->activeIdx = 1 - de->activeIdx;
        de->pendingSwap = false;
        dynEqResetAll(de);
        dynEqApplyConfigLocked(de, &de->cfg[de->activeIdx]);
    }
    else
    {
        de->pendingSwap = true;
        de->pendingIdx = 1 - de->activeIdx;
    }

    pthread_mutex_unlock(&de->cfgLock);
}

void dynamicEq_set_enabled(dynamicEq_t *de, bool enabled)
{
    pthread_mutex_lock(&de->cfgLock);
    if (enabled)
    {
        if (!de->enabled)
        {
            /* Starting from a fully faded state, reset so the first block
             * cannot jump from stale filter state. While a fade is still in
             * progress the filters are live, so leave the state alone. */
            if (de->fade <= 0.0)
            {
                dynEqResetAll(de);
                dynEqApplyConfigLocked(de, &de->cfg[de->activeIdx]);
            }
            de->enabled = true;
        }
    }
    else
    {
        de->enabled = false;
    }
    pthread_mutex_unlock(&de->cfgLock);
}

bool dynamicEq_is_active(dynamicEq_t *de)
{
    pthread_mutex_lock(&de->cfgLock);
    const bool active = de->enabled || de->fade > 0.0 || de->pendingSwap;
    pthread_mutex_unlock(&de->cfgLock);
    return active;
}

void dynamicEq_process_interleaved(dynamicEq_t *de, float *data, size_t numFrames)
{
    if (data == NULL || numFrames == 0)
        return;

    pthread_mutex_lock(&de->cfgLock);

    /* Denormals accumulate in the filters during digital silence and cost
     * far more than the check. Do it once per block, not per sample. */
    for (int k = 0; k < DYNEQ_MAX_BANDS; k++) {
        dyneq_biquad_remove_denormals(&de->state[k].sidechain);
        dyneq_biquad_remove_denormals(&de->state[k].peak[0]);
        dyneq_biquad_remove_denormals(&de->state[k].peak[1]);
    }

    for (size_t i = 0; i < numFrames; i++) {
        /* A pending rate change swaps only when the wet path is silent, so
         * the reset filter state cannot produce a step. */
        if (de->pendingSwap && de->fade <= 0.0) {
            de->activeIdx = de->pendingIdx;
            de->pendingSwap = false;
            dynEqResetAll(de);
            dynEqApplyConfigLocked(de, &de->cfg[de->activeIdx]);
        }

        if (!de->enabled && de->fade <= 0.0)
            break;

        dyneq_cfg_t *cfg = &de->cfg[de->activeIdx];
        if (cfg->bandCount <= 0 || cfg->mix <= 0.0) {
            if (de->fade > 0.0)
                dynEqResetAll(de);
            de->fade = 0.0;
            break;
        }

        if (de->stateVersion != cfg->version)
            dynEqApplyConfigLocked(de, cfg);

        const int numBands = cfg->bandCount;
        const int msMode = cfg->msMode;
        const double mix = cfg->mix;

        const double dryL = data[i * 2];
        const double dryR = data[i * 2 + 1];
        double l, r, mono;

        if (msMode == DYNEQ_MS_STEREO) {
            l = dryL;
            r = dryR;
            /* Sidechain from the sum, so both channels always move together. */
            mono = (dryL + dryR) * 0.5;
        } else {
            /* Working on one half of the stereo picture: the chosen part goes
             * through the filters and is its own sidechain, the other is held
             * aside untouched and put back afterwards. */
            const double mid = (dryL + dryR) * 0.5;
            const double side = (dryL - dryR) * 0.5;
            if (msMode == DYNEQ_MS_MID) { l = mid; r = side; }
            else                        { l = side; r = mid; }
            mono = l;
        }

        for (int k = 0; k < numBands; k++) {
            const dyneq_band_cfg_t *b = &cfg->bands[k];
            if (!b->active)
                continue;

            dyneq_band_state_t *st = &de->state[k];
            const double sc = dyneq_biquad_process(&st->sidechain, mono);
            const double mag = fabs(sc);

            /* Attack when rising, release when falling - a single coefficient
             * would make a fast band chatter and a slow one miss transients. */
            st->env = dynamicEq_env_step(st->env, mag, b->attackCoef, b->releaseCoef);
            if (st->env < DYNEQ_ENV_FLOOR * DYNEQ_ENV_FLOOR)
                st->env = 0.0;

            /* The audible gain follows the target smoothly, so a parameter
             * change while the band is running cannot step the filter. The
             * target itself is refreshed at the redesign tick: the log2 in
             * the gain computer is far too costly per sample. */
            st->appliedDb += (st->gainDb - st->appliedDb) * de->gainSmoothCoef;
            if (st->gainDb == 0.0 && fabs(st->appliedDb) < DYNEQ_EPS_DB)
                st->appliedDb = 0.0;

            l = dyneq_biquad_process(&st->peak[0], l);
            /* In mid or side mode r is the untouched half, so it is carried
             * through rather than filtered. */
            if (msMode == DYNEQ_MS_STEREO)
                r = dyneq_biquad_process(&st->peak[1], r);
        }

        if (++de->redesignCountdown >= DYNEQ_REDESIGN) {
            de->redesignCountdown = 0;
            const double fs = cfg->sampleRate > 0.0 ? cfg->sampleRate : 48000.0;
            for (int k = 0; k < numBands; k++) {
                const dyneq_band_cfg_t *b = &cfg->bands[k];
                dyneq_band_state_t *st = &de->state[k];
                if (!b->active)
                    continue;

                /* Refresh the gain target at the same tick the coefficients
                 * are redesigned at; 0.67 ms at 48 kHz is far shorter than
                 * any useful attack. */
                st->gainDb = dynamicEq_gain_computer_db(st->env, b->thresholdDb,
                                                        b->ratio, b->rangeDb, b->mode);

                /* Snap to the exact identity once the band is at rest, so an
                 * idle band is bit-transparent instead of holding a 0.02 dB
                 * residual forever. */
                if (st->appliedDb == 0.0) {
                    if (st->coeffDb == 0.0)
                        continue;
                    st->coeffDb = 0.0;
                    dynEqSetIdentityPeak(st);
                    continue;
                }
                if (fabs(st->appliedDb - st->coeffDb) <= DYNEQ_EPS_DB)
                    continue;
                double b0, b1, b2, a1, a2;
                st->coeffDb = st->appliedDb;
                dynEqDesignPeaking(b->frequency, b->q, fs, st->coeffDb,
                                   &b0, &b1, &b2, &a1, &a2);
                dynEqSetPeakCoeffs(st, b0, b1, b2, a1, a2);
            }
        }

        double wetL, wetR;
        if (msMode == DYNEQ_MS_STEREO) {
            wetL = l;
            wetR = r;
        } else {
            const double mid = (msMode == DYNEQ_MS_MID) ? l : r;
            const double side = (msMode == DYNEQ_MS_MID) ? r : l;
            wetL = mid + side;
            wetR = mid - side;
        }

        /* Enable/disable and a rate change move the wet contribution through
         * a short fade instead of switching it. */
        const double target = (de->enabled && !de->pendingSwap) ? 1.0 : 0.0;
        de->fade += (target - de->fade) * de->fadeCoef;
        if (target == 0.0 && de->fade < DYNEQ_FADE_SNAP)
            de->fade = 0.0;

        const double wetGain = mix * de->fade;
        if (wetGain >= 1.0) {
            data[i * 2] = (float)wetL;
            data[i * 2 + 1] = (float)wetR;
        } else {
            data[i * 2] = (float)(dryL + (wetL - dryL) * wetGain);
            data[i * 2 + 1] = (float)(dryR + (wetR - dryR) * wetGain);
        }
    }

    pthread_mutex_unlock(&de->cfgLock);
}
