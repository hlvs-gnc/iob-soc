#include "profiling.h"

const char *ProfilingGroupName[PROF_INSTANCES] = {
    "Layer I & II", "Layer III", "Synthesis", "Frame", "Bitstream"};

uint64_t profiling_I_II[LAYER_I_II_NFUNCTIONS];
uint64_t profiling_III[LAYER_III_NFUNCTIONS];
uint64_t profiling_synth[SYNTH_NFUNCTIONS];
uint64_t profiling_frame[FRAME_NFUNCTIONS];
uint64_t profiling_bit[BIT_NFUNCTIONS];

// Global array of profiling groups
ProfilingGroup libmad[PROF_INSTANCES] = {
    {profiling_I_II, LAYER_I_II_NFUNCTIONS},
    {profiling_III, LAYER_III_NFUNCTIONS},
    {profiling_synth, SYNTH_NFUNCTIONS},
    {profiling_frame, FRAME_NFUNCTIONS},
    {profiling_bit, BIT_NFUNCTIONS}};

void initProfiling() {
  for (int i = 0; i < PROF_INSTANCES; i++) {
    resetArray(libmad[i].array, libmad[i].size);
  }
}

void resetArray(uint64_t *profilingGroup, unsigned int size) {
  for (unsigned int i = 0; i < size; i++) {
    profilingGroup[i] = 0;
  }
}

void promptInfo(uint64_t *profilingGroup, unsigned int size, int groupIndex) {
  printf("%s\n", ProfilingGroupName[groupIndex]);
  for (unsigned int i = 0; i < size; i++) {
    printf("Function %u: %llu\n", i + 1, profilingGroup[i]);
  }
  printf("\n");
}