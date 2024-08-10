#include "printf.h"
#include <stdint.h>

#ifndef DECODER_PROFILE
#define DECODER_PROFILE 0

#if (DECODER_PROFILE)
#define PROFILE_I 0
#define PROFILE_II 0
#endif

#define PROF_INSTANCES 5

// LIBMAD function number count
#define LAYER_I_II_NFUNCTIONS 4
#define LAYER_III_NFUNCTIONS 20
#define SYNTH_NFUNCTIONS 4
#define FRAME_NFUNCTIONS 3
#define BIT_NFUNCTIONS 1

extern uint64_t profiling_I_II[LAYER_I_II_NFUNCTIONS];
extern uint64_t profiling_III[LAYER_III_NFUNCTIONS];
extern uint64_t profiling_synth[SYNTH_NFUNCTIONS];
extern uint64_t profiling_frame[FRAME_NFUNCTIONS];
extern uint64_t profiling_bit[BIT_NFUNCTIONS];

// ProfilingGroup structure
typedef struct {
  uint64_t *array;
  unsigned int size;
} ProfilingGroup;

extern ProfilingGroup libmad[PROF_INSTANCES];

void initProfiling();
void resetArray(uint64_t *profilingGroup, unsigned int size);
void promptInfo(uint64_t *profilingGroup, unsigned int size, int groupIndex);