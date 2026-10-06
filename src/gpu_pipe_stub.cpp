#include "../include/gpu_pipe.h"
#include "../include/gpu_gas.h"
#include "../include/gpu_chamber.h"
#include "../include/gpu_coupled.h"
#include <cstdlib>
#include <stdexcept>
namespace gpu_pipe {
bool enabled() {
    // The environment is fixed at launch, so read it once. This is called from
    // the innermost CFL substep loop (piston_engine_simulator.cpp), where a CRT
    // getenv costs ~2 us per substep across the whole run. The throw below is
    // deliberately kept outside the cached initialiser so it still fires on
    // every call, exactly as before.
    static const bool gpuRequested = [] {
        const char *value=std::getenv("ENGINE_SIM_GPU");
        return value && value[0]=='1';
    }();
    if (gpuRequested)
        throw std::runtime_error("ENGINE_SIM_GPU=1 requires a CUDA-enabled build");
    return false;
}
const char *deviceName() { return "CPU"; }
void advance(Pipe *, int, double) { throw std::runtime_error("CUDA backend is not built"); }
}
namespace gpu_gas {
void advance(Transfer *,int) { throw std::runtime_error("CUDA backend is not built"); }
}

namespace gpu_chamber {
void advance(Cylinder *,int,double) { throw std::runtime_error("CUDA backend is not built"); }
}

namespace gpu_coupled {
bool requested() { return false; }
bool advance(Batch &,double,int) { throw std::runtime_error("CUDA backend is not built"); }
}
