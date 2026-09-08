#ifndef ENGINE_SIM_GPU_PIPE_H
#define ENGINE_SIM_GPU_PIPE_H

namespace gpu_pipe {
// Conserved quantities per volume: fuel, O2, N2, CO2, H2O, momentum,
// total energy, burned-gas mass. The CUDA solver uses double throughout.
struct Pipe {
    double u[64][8];
    double dx, diameter, friction, fuelMass, timestep;
    int count;
};
bool enabled();
const char *deviceName();
void advance(Pipe *pipes, int count, double dt);
}
#endif
