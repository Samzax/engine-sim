#ifndef ENGINE_SIM_GPU_GAS_H
#define ENGINE_SIM_GPU_GAS_H

#include "gas_system.h"
#include <type_traits>

namespace gpu_gas {
// Independent transfers for validation and development of coupled GPU flow.
// Live engine port scheduling does not use this standalone batch yet.
struct Transfer {
    GasSystem a,b;
    double conductance=0,dt=0,directionX=1,directionY=0,areaA=0,areaB=0;
    double transferredMoles=0;
    bool fixedEnvironment=false; // Treat b as an unchanged pressure/temperature reservoir.
};
static_assert(std::is_trivially_copyable<Transfer>::value,"CUDA transfer state must be trivially copyable");
static_assert(std::is_standard_layout<GasSystem>::value,"CUDA gas layout must be standard");
void advance(Transfer *transfers,int count);
}
#endif
