#ifndef ENGINE_SIM_GPU_COUPLED_H
#define ENGINE_SIM_GPU_COUPLED_H
#include "chamber_flow.h"
#include "reservoir_flow.h"
#include "gpu_pipe.h"
#include <vector>

namespace gpu_coupled {
struct Pipe { gpu_pipe::Pipe solver{}; int firstCell=0; };
struct Intake { reservoir_flow::IntakeState state; reservoir_flow::IntakeParameters parameters; };
struct Exhaust { reservoir_flow::ExhaustState state; reservoir_flow::ExhaustParameters parameters; };
struct Cylinder {
    chamber_flow::State state;
    chamber_flow::Parameters parameters;
    int intake=0,exhaust=0,intakePipe=0,exhaustPipe=0;
    double manifoldK=0,collectorK=0;
};
struct Batch {
    std::vector<GasSystem> cells;
    std::vector<Pipe> pipes;
    std::vector<Cylinder> cylinders;
    std::vector<Intake> intakes;
    std::vector<Exhaust> exhausts;
};
bool requested();
// Returns false when cooperative launch cannot support this batch on the device.
// Successful calls publish the whole mechanical step after device completion.
bool advance(Batch &batch,double timestep,int fluidSteps);
}
#endif
