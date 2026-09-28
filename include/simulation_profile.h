#ifndef ENGINE_SIM_SIMULATION_PROFILE_H
#define ENGINE_SIM_SIMULATION_PROFILE_H

#ifdef ENGINE_SIM_PROFILE
#include <chrono>
#include <cstdio>
#include <cstdint>
namespace simulation_profile {
enum Phase {Step, Fluid, Reservoirs, Cfl, Ports, Pipes, DeviceWait,
    ReservoirFlow, PipeScan, ChainFlow, CylStage, ChamberBegin, ChamberFlow,
    ChamberFinish, Count};
struct Counters {
    double seconds[Count]{};
    uint64_t calls[Count]{};
    ~Counters() {
        const char *names[Count]={"step","fluid","reservoirs","cfl","ports","pipes","device_wait",
            "reservoir_flow","pipe_scan","chain_flow","cyl_stage","chamber_begin","chamber_flow",
            "chamber_finish"};
        for(int i=0;i<Count;++i) if(calls[i])
            std::fprintf(stderr,"PROFILE inclusive %s calls=%llu seconds=%.9f\n",names[i],
                static_cast<unsigned long long>(calls[i]),seconds[i]);
    }
};
inline Counters &counters() {thread_local Counters value;return value;}
struct ThermoCounters {
    uint64_t newtonCalls=0, newtonIters=0, refreshProps=0, flowParams=0;
    ~ThermoCounters() {
        if (newtonCalls||newtonIters||refreshProps||flowParams)
            std::fprintf(stderr,"THERMO newton_calls=%llu newton_iters=%llu refresh=%llu flow=%llu\n",
                static_cast<unsigned long long>(newtonCalls),
                static_cast<unsigned long long>(newtonIters),
                static_cast<unsigned long long>(refreshProps),
                static_cast<unsigned long long>(flowParams));
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
