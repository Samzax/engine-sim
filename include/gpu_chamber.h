#ifndef ENGINE_SIM_GPU_CHAMBER_H
#define ENGINE_SIM_GPU_CHAMBER_H
#include "chamber_flow.h"
#include <type_traits>

namespace gpu_chamber {
// Independent cylinder stages, after reservoir ports and before pipe interiors.
// This validation interface is not the live coupled-engine scheduler.
struct Cylinder {
    chamber_flow::State state;
    chamber_flow::Parameters parameters;
    GasSystem intake, exhaust;
};
static_assert(std::is_trivially_copyable<Cylinder>::value,"CUDA cylinder state must be trivially copyable");
void advance(Cylinder *cylinders,int count,double dt);
}
#endif
