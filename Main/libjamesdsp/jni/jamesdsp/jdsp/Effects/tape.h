#ifndef TAPE_H
#define TAPE_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Vintage tape: wow, flutter, saturation, bias and head bump.
 *
 * Ported from RootlessViPER4Android's tape.c (GPL-3.0), reworked as a
 * standalone module like dynamicEq.c/lufs_normalizer.h. Differences from
 * the source:
 *  - interleaved stereo processing instead of the planar tmpBuffer pair;
 *  - the switch-off fade is requested by tape_set_enabled(false) and must
 *    be drained by the caller: keep calling tape_process_interleaved while
 *    tape_is_active() is true, or the wet signal will cut off with a click
 *    (the source used the jdsp->tapeEnabled flag for the same purpose);
 *  - parameters are clamped here, at the IPC trust boundary;
 *  - tape_refresh() recomputes the rate-dependent design on sample-rate
 *    change, same as the source's TapeRefresh.
 */

#define TAPE_LINE 8192

typedef struct
{
    float b0, b1, b2, a1, a2;
    float z1[2], z2[2];
} tape_stage_t;

typedef struct
{
    bool enabled;
    /* Parameters, as the user gave them */
    double sampleRate;
    float wowPct, flutterPct;
    float saturation, bias, headBumpDb, mix;
    /* Derived: depths in samples, increments in radians per sample, base
       read offset and fade length in samples. Only meaningful at one rate;
       tape_refresh redoes them when the rate changes. */
    float base, wowPhase, wowInc, wowDepth;
    float flutterPhase, flutterInc, flutterDepth;
    float drive;
    tape_stage_t biasShelf, headBump;
    int fadeLen;
    /* State */
    float line[2][TAPE_LINE];
    int widx;
    /* `filled` counts samples written until there is at least the read
       offset in the line; `wetGain` then walks towards 1 or 0 over
       `fadeLen`. `fadingOut` means a switch-off is in progress. */
    int filled, fadingOut;
    float wetGain;
    int transparent;
} tape_t;

void tape_init(tape_t *t);
void tape_configure(tape_t *t, double sampleRate, float wowPct, float flutterPct,
                    float saturationPct, float biasPct, float headBumpDb,
                    float mixPct);
void tape_refresh(tape_t *t, double sampleRate);
void tape_set_enabled(tape_t *t, bool enabled);
bool tape_is_active(const tape_t *t);
void tape_process_interleaved(tape_t *t, float *data, size_t numFrames);

#ifdef __cplusplus
}
#endif

#endif
