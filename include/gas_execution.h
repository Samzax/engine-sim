#ifndef ENGINE_SIM_GAS_EXECUTION_H
#define ENGINE_SIM_GAS_EXECUTION_H

// gas_system.cpp is compiled by the C++ compiler for CPU execution and included
// by the CUDA translation unit for device execution. Inline device definitions
// keep NVCC's unused aborting host stubs from conflicting with CPU definitions.
// The value type's defaulted special members are inferred
// by NVCC and its layout contains no pointers or virtual functions.
#ifdef __CUDACC__
#define ES_GAS_FUNCTION __device__
#define ES_GAS_DEFINITION __device__ inline
#else
#define ES_GAS_FUNCTION
#define ES_GAS_DEFINITION
#endif

#endif
