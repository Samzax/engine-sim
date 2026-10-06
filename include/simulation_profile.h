#ifndef ENGINE_SIM_SIMULATION_PROFILE_H
#define ENGINE_SIM_SIMULATION_PROFILE_H

#ifdef ENGINE_SIM_PROFILE
#include <chrono>
#include <cstdio>
#include <cstdint>
namespace simulation_profile {
enum Phase {Step, Fluid, Reservoirs, Cfl, Ports, Pipes, DeviceWait,
    ReservoirFlow, PipeScan, ChainFlow, CylStage, ChamberBegin, ChamberFlow,
    ChamberFinish, ChamberUpdate, Ignite, Aggregate, Rigid, Mechanics,
    AudioWrite, Count};
struct Counters {
    double seconds[Count]{};
    uint64_t calls[Count]{};
    ~Counters() {
        const char *names[Count]={"step","fluid","reservoirs","cfl","ports","pipes","device_wait",
            "reservoir_flow","pipe_scan","chain_flow","cyl_stage","chamber_begin","chamber_flow",
            "chamber_finish","chamber_update","ignite","aggregate","rigid","mechanics",
            "audio_write"};
        for(int i=0;i<Count;++i) if(calls[i])
            std::fprintf(stderr,"PROFILE inclusive %s calls=%llu seconds=%.9f\n",names[i],
                static_cast<unsigned long long>(calls[i]),seconds[i]);
    }
};
inline Counters &counters() {thread_local Counters value;return value;}
struct ThermoCounters {
    uint64_t newtonCalls=0, newtonIters=0, refreshProps=0, flowParams=0;
    uint64_t tempQueries=0, tempHits=0, invalidates=0, restores=0, restoreNoOp=0;
    // Batch B: where the fluid time actually goes. All are thread_local; sum
    // the per-thread lines when reading them.
    uint64_t bisectionCalls=0, trialCopies=0;   // GasSystem::flow sign bisection
    uint64_t cflCacheHits=0, cflCacheRecomputes=0; // GasPipe::stableTimestep cache
    uint64_t advanceCalls=0, advanceSubsteps=0; // GasPipe::advance substep loop
    uint64_t exchangeCalls=0, exchangeIters=0;  // CylinderThermalModel::exchange
    uint64_t fluidSlices=0, cflBinds=0;         // piston sim fluid-step slices
    ~ThermoCounters() {
        if (newtonCalls||newtonIters||refreshProps||flowParams||tempQueries||restores||invalidates
            ||bisectionCalls||cflCacheHits||advanceCalls||exchangeCalls||fluidSlices)
            std::fprintf(stderr,
                "THERMO newton_calls=%llu newton_iters=%llu refresh=%llu flow=%llu queries=%llu hits=%llu invalidates=%llu restores=%llu restore_noop=%llu\n"
                "THERMO2 bisection_calls=%llu trial_copies=%llu cfl_cache_hits=%llu cfl_cache_recomputes=%llu advance_calls=%llu advance_substeps=%llu exchange_calls=%llu exchange_iters=%llu fluid_slices=%llu cfl_binds=%llu\n",
                static_cast<unsigned long long>(newtonCalls),
                static_cast<unsigned long long>(newtonIters),
                static_cast<unsigned long long>(refreshProps),
                static_cast<unsigned long long>(flowParams),
                static_cast<unsigned long long>(tempQueries),
                static_cast<unsigned long long>(tempHits),
                static_cast<unsigned long long>(invalidates),
                static_cast<unsigned long long>(restores),
                static_cast<unsigned long long>(restoreNoOp),
                static_cast<unsigned long long>(bisectionCalls),
                static_cast<unsigned long long>(trialCopies),
                static_cast<unsigned long long>(cflCacheHits),
                static_cast<unsigned long long>(cflCacheRecomputes),
                static_cast<unsigned long long>(advanceCalls),
                static_cast<unsigned long long>(advanceSubsteps),
                static_cast<unsigned long long>(exchangeCalls),
                static_cast<unsigned long long>(exchangeIters),
                static_cast<unsigned long long>(fluidSlices),
                static_cast<unsigned long long>(cflBinds));
    }
};
inline ThermoCounters &thermo() {thread_local ThermoCounters value;return value;}
struct Scope {
    Phase phase;
    std::chrono::steady_clock::time_point start;
    explicit Scope(Phase p):phase(p),start(std::chrono::steady_clock::now()) {}
    ~Scope() {
        auto &c=counters();
        c.seconds[phase]+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        ++c.calls[phase];
    }
};
}
#define ENGINE_SIM_PROFILE_JOIN_(a,b) a##b
#define ENGINE_SIM_PROFILE_JOIN(a,b) ENGINE_SIM_PROFILE_JOIN_(a,b)
#define ENGINE_SIM_PROFILE_SCOPE(phase) simulation_profile::Scope ENGINE_SIM_PROFILE_JOIN(profileScope_,__LINE__)(simulation_profile::phase)
#else
#define ENGINE_SIM_PROFILE_SCOPE(phase) ((void)0)
#endif
#endif
