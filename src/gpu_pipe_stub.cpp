#include "../include/gpu_pipe.h"
#include "../include/gpu_gas.h"
#include "../include/gpu_chamber.h"
#include <cstdlib>
#include <stdexcept>
namespace gpu_pipe {
bool enabled() {
    const char *value=std::getenv("ENGINE_SIM_GPU");
    if (value && value[0]=='1')
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
