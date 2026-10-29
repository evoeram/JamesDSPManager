#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "../jdsp_header.h"
void ArbitraryResponseEqualizerConstructor(JamesDSPLib *jdsp)
{
	jdsp->arbMag.instance.filterLen = InitArbitraryEq(&jdsp->arbMag.instance.coeffGen, 0);
	FFTConvolver2x2Init(&jdsp->arbMag.instance.convState);
	FFTConvolver2x2Init(&jdsp->arbMag.conv);
	float *kDelta = (float*)malloc(jdsp->arbMag.instance.filterLen * sizeof(float));
	memset(kDelta, 0, jdsp->arbMag.instance.filterLen * sizeof(float));
	kDelta[0] = 1.0f;
	FFTConvolver2x2LoadImpulseResponse(&jdsp->arbMag.instance.convState, (unsigned int)jdsp->blockSize, kDelta, kDelta, jdsp->arbMag.instance.filterLen);
	FFTConvolver2x2LoadImpulseResponse(&jdsp->arbMag.conv, (unsigned int)jdsp->blockSize, kDelta, kDelta, jdsp->arbMag.instance.filterLen);
	free(kDelta);
}
void ArbitraryResponseEqualizerDestructor(JamesDSPLib *jdsp)
{
	EqNodesFree(&jdsp->arbMag.instance.coeffGen);
	FFTConvolver2x2Free(&jdsp->arbMag.instance.convState);
	FFTConvolver2x2Free(&jdsp->arbMag.conv);
}
/* The parser runs on the settings thread and takes the lock for the whole
   job: it rewrites the generator's nodes and the convolver's segments that
   the audio thread runs, and the audio thread also rebuilds the convolver
   on a block size change (through Enable). */
void ArbitraryResponseEqualizerStringParser(JamesDSPLib *jdsp, char *stringEq)
{
	jdsp_lock(jdsp);
	ArbitraryEqString2SortedNodes(&jdsp->arbMag.instance.coeffGen, stringEq);
	float *eqFil = jdsp->arbMag.instance.coeffGen.GetFilter(&jdsp->arbMag.instance.coeffGen, (float)jdsp->fs);
	// A refresh rewrites the loaded segments in place, which is only right
	// while the filter needs the same number of segments as the one loaded
	// (the phase switch in the FIR equalizer changes the filter length).
	// When the count differs, load in full, the same way the deferred
	// refresh in Enable does.
	const unsigned int bs = jdsp->arbMag.instance.convState._blockSize;
	if (bs && (jdsp->arbMag.instance.filterLen + bs - 1) / bs == jdsp->arbMag.instance.convState._segCount)
		FFTConvolver2x2RefreshImpulseResponse(&jdsp->arbMag.instance.convState, &jdsp->arbMag.conv, eqFil, eqFil, jdsp->arbMag.instance.filterLen);
	else
	{
		FFTConvolver2x2Free(&jdsp->arbMag.instance.convState);
		FFTConvolver2x2Free(&jdsp->arbMag.conv);
		FFTConvolver2x2LoadImpulseResponse(&jdsp->arbMag.instance.convState, (unsigned int)jdsp->blockSize, eqFil, eqFil, jdsp->arbMag.instance.filterLen);
		FFTConvolver2x2LoadImpulseResponse(&jdsp->arbMag.conv, (unsigned int)jdsp->blockSize, eqFil, eqFil, jdsp->arbMag.instance.filterLen);
	}
	jdsp_unlock(jdsp);
}
void ArbitraryResponseEqualizerEnable(JamesDSPLib *jdsp, char enable)
{
	if (jdsp->arbMagForceRefresh)
	{
		/* The kept filters and the generator are chosen under the lock too:
		   the settings thread frees and replaces both. */
		jdsp_lock(jdsp);
		float *eqFil = jdsp->arbMag.instance.coeffGen.GetFilter(&jdsp->arbMag.instance.coeffGen, (float)jdsp->fs);
		FFTConvolver2x2Free(&jdsp->arbMag.instance.convState);
		FFTConvolver2x2Free(&jdsp->arbMag.conv);
		FFTConvolver2x2LoadImpulseResponse(&jdsp->arbMag.instance.convState, (unsigned int)jdsp->blockSize, eqFil, eqFil, jdsp->arbMag.instance.filterLen);
		FFTConvolver2x2LoadImpulseResponse(&jdsp->arbMag.conv, (unsigned int)jdsp->blockSize, eqFil, eqFil, jdsp->arbMag.instance.filterLen);
		jdsp_unlock(jdsp);
		jdsp->arbMagForceRefresh = 0;
	}
	if (enable)
		jdsp->arbitraryMagEnabled = 1;
}
void ArbitraryResponseEqualizerDisable(JamesDSPLib *jdsp)
{
	jdsp->arbitraryMagEnabled = 0;
}
void ArbitraryResponseEqualizerProcess(JamesDSPLib *jdsp, size_t n)
{
	FFTConvolver2x2Process(&jdsp->arbMag.conv, jdsp->tmpBuffer[0], jdsp->tmpBuffer[1], jdsp->tmpBuffer[0], jdsp->tmpBuffer[1], (unsigned int)n);
}