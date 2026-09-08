#include "../include/gpu_pipe.h"
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
