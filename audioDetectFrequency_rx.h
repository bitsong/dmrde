#ifndef _AUDIO_SIGNAL_RX
#define _AUDIO_SIGNAL_RX
#include <xdc/std.h>
#include "audio_queue.h"




static Float calculatAmp(audioQueue *q, Float *outBuf);
static void IF_Filter(Float *deDcBufOut, Float *ifBufOut);
static void phaseDetector(Float *ifBufOut);
static Float oneCalaculate(Float x, const Float a[], const Float b[], Float reg[]);
static void lowFreqFilter(Float *ifBufOut, Float *lfBufOut);
//static void freqDetector(Float *input, Float *output);
static void deEmphasis(Float * lfBufOut, Float * deEmBufOut);
static void AF_Filter(Float *deEmBufOut, Float *afBufOut);
void audioSignalRX();


#endif
