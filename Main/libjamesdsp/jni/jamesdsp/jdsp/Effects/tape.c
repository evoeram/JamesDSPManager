/* tape.c — Vintage tape: wow, flutter, saturation, bias, head bump.
 * Standalone module ported from RootlessViPER4Android's tape.c.
 * See tape.h for the porting notes. */

#include "tape.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Base read offset, in samples at 48k, scaled with the rate. The line has to
   sit far enough back that the modulation never runs off either end. */
#define TAPE_BASE_MS 12.0f

/* How long the wet signal takes to arrive once the line has something in it. */
#define TAPE_FADE_MS 10.0f

static void tapeDesignShelf(tape_stage_t *s, double f, float fs, float gainDb, int high)
{
    if (fabsf(gainDb) < 1e-6f)
    {
        s->b0 = 1.0f; s->b1 = 0.0f; s->b2 = 0.0f; s->a1 = 0.0f; s->a2 = 0.0f;
        return;
    }
    const double A = pow(10.0, (double)gainDb / 40.0);
    const double w0 = 2.0 * M_PI * f / (double)fs;
    const double cw = cos(w0), sw = sin(w0);
    const double alpha = sw / (2.0 * 0.70710678);
    const double sq = 2.0 * sqrt(A) * alpha;
    double b0, b1, b2, a0, a1, a2;
    if (high)
    {
        b0 = A * ((A + 1.0) + (A - 1.0) * cw + sq);
        b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cw);
        b2 = A * ((A + 1.0) + (A - 1.0) * cw - sq);
        a0 = (A + 1.0) - (A - 1.0) * cw + sq;
        a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cw);
        a2 = (A + 1.0) - (A - 1.0) * cw - sq;
    }
    else
    {
        b0 = A * ((A + 1.0) - (A - 1.0) * cw + sq);
        b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cw);
        b2 = A * ((A + 1.0) - (A - 1.0) * cw - sq);
        a0 = (A + 1.0) + (A - 1.0) * cw + sq;
        a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cw);
        a2 = (A + 1.0) + (A - 1.0) * cw - sq;
    }
    s->b0 = (float)(b0 / a0); s->b1 = (float)(b1 / a0); s->b2 = (float)(b2 / a0);
    s->a1 = (float)(a1 / a0); s->a2 = (float)(a2 / a0);
}

static void tapeDesignPeak(tape_stage_t *s, double f, float fs, float gainDb, double q)
{
    if (fabsf(gainDb) < 1e-6f)
    {
        s->b0 = 1.0f; s->b1 = 0.0f; s->b2 = 0.0f; s->a1 = 0.0f; s->a2 = 0.0f;
        return;
    }
    const double A = pow(10.0, (double)gainDb / 40.0);
    const double w0 = 2.0 * M_PI * f / (double)fs;
    const double alpha = sin(w0) / (2.0 * q);
    const double a0 = 1.0 + alpha / A;
    s->b0 = (float)((1.0 + alpha * A) / a0);
    s->b1 = (float)((-2.0 * cos(w0)) / a0);
    s->b2 = (float)((1.0 - alpha * A) / a0);
    s->a1 = (float)((-2.0 * cos(w0)) / a0);
    s->a2 = (float)((1.0 - alpha / A) / a0);
}

static float tapeRun(tape_stage_t *s, int ch, float x)
{
    const float y = s->b0 * x + s->z1[ch];
    s->z1[ch] = s->b1 * x - s->a1 * y + s->z2[ch];
    s->z2[ch] = s->b2 * x - s->a2 * y;
    return y;
}

/* Depth formulas are from the source: real machines wobble by a fraction of
   a percent; these go further so the control is usable as an effect. */
static void tapeDesign(tape_t *t, float fs)
{
    t->wowDepth = (t->wowPct * 0.01f) * 0.0035f * fs;
    t->flutterDepth = (t->flutterPct * 0.01f) * 0.00035f * fs;
    t->wowInc = 2.0f * (float)M_PI * 0.7f / fs;
    t->flutterInc = 2.0f * (float)M_PI * 9.0f / fs;

    /* Under-biasing gives MORE top end (with more distortion alongside) and
       over-biasing dulls the sound, so a negative setting lifts the shelf.
       Measured at 12 kHz in the source: -100 reads +5.95 dB, +100 -5.95 dB. */
    tapeDesignShelf(&t->biasShelf, 4500.0, fs, -(t->bias * 0.01f) * 6.0f, 1);
    tapeDesignPeak(&t->headBump, 60.0, fs, t->headBumpDb, 1.1);

    t->base = TAPE_BASE_MS * 0.001f * fs;
    if (t->base < 4.0f) t->base = 4.0f;
    if (t->base > (float)(TAPE_LINE - 4)) t->base = (float)(TAPE_LINE - 4);

    t->fadeLen = (int)(TAPE_FADE_MS * 0.001f * fs);
    if (t->fadeLen < 1) t->fadeLen = 1;
    t->sampleRate = fs;
}

static inline float tapeRead(const float *line, int widx, float offset)
{
    float readPos = (float)widx - offset;
    while (readPos < 0.0f) readPos += (float)TAPE_LINE;
    const int ip = (int)readPos;
    const float frac = readPos - (float)ip;
    /* Masked, both of them: the add above can round up to exactly TAPE_LINE.
       TAPE_LINE wraps to 0, which is the same place. */
    const int i0 = ip & (TAPE_LINE - 1);
    const int i1 = (ip + 1) & (TAPE_LINE - 1);
    return line[i0] * (1.0f - frac) + line[i1] * frac;
}

void tape_init(tape_t *t)
{
    memset(t, 0, sizeof(*t));
    /* Source defaults (jdspController.c): wow 25, flutter 30, saturation 35,
       bias -20, head bump 3 dB, mix 100. */
    tape_configure(t, 48000.0, 25.0f, 30.0f, 35.0f, -20.0f, 3.0f, 100.0f);
    t->enabled = false;
}

void tape_configure(tape_t *t, double sampleRate, float wowPct, float flutterPct,
                    float saturationPct, float biasPct, float headBumpDb,
                    float mixPct)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    const float fs = (float)sampleRate;

    /* Was it doing nothing before this call? If so, process has been
       returning early and the line has been going stale, so coming back out
       of that needs the same treatment as switching the card on. */
    const int wasIdle = t->transparent || t->mix <= 0.0f;

    if (wowPct < 0.0f) wowPct = 0.0f;
    if (wowPct > 100.0f) wowPct = 100.0f;
    t->wowPct = wowPct;
    if (flutterPct < 0.0f) flutterPct = 0.0f;
    if (flutterPct > 100.0f) flutterPct = 100.0f;
    t->flutterPct = flutterPct;

    if (saturationPct < 0.0f) saturationPct = 0.0f;
    if (saturationPct > 100.0f) saturationPct = 100.0f;
    t->saturation = saturationPct * 0.01f;
    /* Drive rises with the control so more of it means more harmonics, not
       just more of the same shaped signal. */
    t->drive = 1.0f + t->saturation * 7.0f;

    if (biasPct < -100.0f) biasPct = -100.0f;
    if (biasPct > 100.0f) biasPct = 100.0f;
    t->bias = biasPct;

    if (headBumpDb < 0.0f) headBumpDb = 0.0f;
    if (headBumpDb > 9.0f) headBumpDb = 9.0f;
    t->headBumpDb = headBumpDb;

    tapeDesign(t, fs);

    if (mixPct < 0.0f) mixPct = 0.0f;
    if (mixPct > 100.0f) mixPct = 100.0f;
    t->mix = mixPct * 0.01f;

    /* Nothing asked for means nothing done - and it has to be a real early
       return, because the delay line alone would shift the whole signal
       twelve milliseconds even with no modulation on it. */
    t->transparent = wowPct <= 0.0f && flutterPct <= 0.0f &&
        saturationPct <= 0.0f && fabsf(biasPct) < 1e-6f && headBumpDb <= 0.0f;

    if (wasIdle && !t->transparent && t->mix > 0.0f)
    {
        t->filled = 0;
        t->wetGain = 0.0f;
    }
}

void tape_refresh(tape_t *t, double sampleRate)
{
    if (!isfinite(sampleRate) || sampleRate <= 0.0)
        sampleRate = 48000.0;
    /* The delay line and the oscillator phases are left alone: the read
       offset moves to the new rate's equivalent of twelve milliseconds,
       which is a small step rather than the click that clearing the line
       would give. */
    tapeDesign(t, (float)sampleRate);
}

void tape_set_enabled(tape_t *t, bool enabled)
{
    if (enabled)
    {
        if (t->enabled)
        {
            /* Already on, or a switch-off is still fading: the line is still
               full and current, so cancel the fade instead of clearing it
               and making the user wait through another fill. */
            t->fadingOut = 0;
            return;
        }
        if (!t->fadingOut)
        {
            /* Fully off: the line holds whatever was in it when the fade
               finished - possibly minutes ago. Treat it as empty again. */
            memset(t->line, 0, sizeof(t->line));
            t->widx = 0;
            t->filled = 0;
            t->wetGain = 0.0f;
            t->wowPhase = 0.0f;
            t->flutterPhase = 0.0f;
            t->biasShelf.z1[0] = t->biasShelf.z2[0] = 0.0f;
            t->biasShelf.z1[1] = t->biasShelf.z2[1] = 0.0f;
            t->headBump.z1[0] = t->headBump.z2[0] = 0.0f;
            t->headBump.z1[1] = t->headBump.z2[1] = 0.0f;
        }
        t->fadingOut = 0;
        t->enabled = true;
    }
    else
    {
        if (!t->enabled)
            return;
        /* The flag is cleared here, but the fade is only requested: process
           takes the wet signal down over the same few milliseconds it uses
           coming in, and clears fadingOut once it arrives at nothing. */
        t->enabled = false;
        t->fadingOut = 1;
    }
}

bool tape_is_active(const tape_t *t)
{
    return t->enabled || t->fadingOut || t->wetGain > 0.0f;
}

void tape_process_interleaved(tape_t *t, float *data, size_t numFrames)
{
    if (!t->enabled && !t->fadingOut)
        return;
    if (t->mix <= 0.0f || t->transparent)
    {
        /* Nothing audible to fade, so a pending switch-off can finish here. */
        if (t->fadingOut)
        {
            t->fadingOut = 0;
            t->wetGain = 0.0f;
        }
        return;
    }

    const float inv = 1.0f / t->drive;

    for (size_t i = 0; i < numFrames; i++)
    {
        const float dry[2] = { data[i * 2], data[i * 2 + 1] };

        /* Nothing until the line holds at least the read offset - switching
           on mid-track otherwise reads twelve milliseconds of the zeroes
           enable left behind. Then in over a few milliseconds, because
           arriving all at once at the moment the line happens to be full is
           just a later click. At wetAmount 0 the output is exactly dry, bit
           for bit, which is what makes the fill period inaudible. */
        float wetAmount;
        if (t->filled < (int)t->base)
        {
            t->filled++;
            t->wetGain = 0.0f;
            wetAmount = 0.0f;
        }
        else
        {
            const float step = 1.0f / (float)t->fadeLen;
            const float target = t->fadingOut ? 0.0f : 1.0f;
            if (t->wetGain < target)
            {
                t->wetGain += step;
                if (t->wetGain > target) t->wetGain = target;
            }
            else if (t->wetGain > target)
            {
                t->wetGain -= step;
                if (t->wetGain < target) t->wetGain = target;
            }
            wetAmount = t->mix * t->wetGain;
        }

        /* One pair of oscillators drives both channels, so the two sides
           drift together; independent ones would swing the image about,
           which is a chorus rather than a worn machine. */
        t->wowPhase += t->wowInc;
        if (t->wowPhase > 2.0f * (float)M_PI) t->wowPhase -= 2.0f * (float)M_PI;
        t->flutterPhase += t->flutterInc;
        if (t->flutterPhase > 2.0f * (float)M_PI) t->flutterPhase -= 2.0f * (float)M_PI;

        const float offset = t->base
            + t->wowDepth * sinf(t->wowPhase)
            + t->flutterDepth * sinf(t->flutterPhase);

        for (int ch = 0; ch < 2; ch++)
        {
            t->line[ch][t->widx] = dry[ch];

            /* Read at a fractional distance behind the write head. */
            float y = tapeRead(t->line[ch], t->widx, offset);

            if (t->saturation > 0.0f)
            {
                /* Runs out of headroom gently and never quite limits. Unity
                   slope at the origin so the control adds harmonics rather
                   than level. */
                const float xd = y * t->drive;
                const float sat = (xd / (1.0f + fabsf(xd))) * inv;
                y = y * (1.0f - t->saturation) + sat * t->saturation;
            }

            y = tapeRun(&t->biasShelf, ch, y);
            y = tapeRun(&t->headBump, ch, y);

            if (wetAmount >= 1.0f) data[i * 2 + ch] = y;
            else data[i * 2 + ch] = dry[ch] + (y - dry[ch]) * wetAmount;
        }

        t->widx = (t->widx + 1) & (TAPE_LINE - 1);
    }

    /* The switch-off has run its course, so the caller can stop feeding us. */
    if (t->fadingOut && t->wetGain <= 0.0f)
        t->fadingOut = 0;
}
