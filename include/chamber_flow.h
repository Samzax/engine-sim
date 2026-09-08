#ifndef ENGINE_SIM_CHAMBER_FLOW_H
#define ENGINE_SIM_CHAMBER_FLOW_H
#include "gas_system.h"
#include "cylinder_thermal_model.h"

// Numerical chamber stages shared by CPU scheduling and the coupled CUDA path.
// Mechanics, ignition and dynamic flame-speed preparation occur before these.
namespace chamber_flow {
struct FlameEvent {
    double lit_n = 0;
    double total_n = 0;
    double percentageLit = 0;
    double efficiency = 1.0;
    double flameSpeed = 0.0;
    double ignitionTemperature = 300, ignitionPressure = 101325;
    double unburnedTemperature = 300;

    double lastVolume = 0.0;
    double travel_x = 0.0;
    double travel_y = 0.0;
    GasSystem::Mix globalMix;
};


struct Parameters {
    double volume, cylinderHeight, surfaceArea, bore, boreArea, meanPistonSpeed;
    double blowbyK, crankcasePressure, intakeK, exhaustK, intakeArea, exhaustArea;
    double fuelMass, fuelEnergyDensity;
};
struct State {
    GasSystem system;
    CylinderThermalModel thermal;
    FlameEvent flame;
    bool lit=false;
    double peakTemperature=0, burntFuel=0, exhaustFlow=0, totalExhaustFlow=0, totalIntakeFlow=0;
};
// References let CPU stages reuse their existing state without per-substep copies.
struct View {
    GasSystem &system;
    CylinderThermalModel &thermal;
    FlameEvent &flame;
    bool &lit;
    double &peakTemperature, &burntFuel, &exhaustFlow, &totalExhaustFlow, &totalIntakeFlow;
};
ES_GAS_FUNCTION inline View view(State &s) {
    return {s.system,s.thermal,s.flame,s.lit,s.peakTemperature,s.burntFuel,
        s.exhaustFlow,s.totalExhaustFlow,s.totalIntakeFlow};
}
ES_GAS_FUNCTION inline void begin(View s,const Parameters &p,double dt) {
    if(s.system.temperature()>s.peakTemperature) s.peakTemperature=s.system.temperature();
    s.thermal.exchange(s.system,p.surfaceArea,p.meanPistonSpeed,dt);
    s.system.flow(p.blowbyK,dt,p.crankcasePressure,units::celcius(25.0));
}
ES_GAS_FUNCTION inline void finish(View s,const Parameters &p,double intakeFlow,double exhaustFlow,double dt) {
    if (std::abs(intakeFlow) > 1E-9 && s.lit) {
        s.lit = false;
    }

    s.exhaustFlow = exhaustFlow;
    s.totalExhaustFlow += exhaustFlow;
    s.totalIntakeFlow += intakeFlow;

    if (s.lit) {
        const double totalTravel_x = p.bore / 2;
        const double totalTravel_y = p.volume / p.boreArea;
        const double expansion = p.volume / s.flame.lastVolume;
        const double lastTravel_x = s.flame.travel_x;
        const double lastTravel_y = s.flame.travel_y * expansion;
        const double flameSpeed = s.flame.flameSpeed;

        s.flame.travel_x =
            std::fmin(lastTravel_x + dt * flameSpeed, totalTravel_x);
        s.flame.travel_y =
            std::fmin(lastTravel_y + dt * flameSpeed, totalTravel_y);

        if (lastTravel_x < s.flame.travel_x || lastTravel_y < s.flame.travel_y) {
            const double burnedVolume =
                s.flame.travel_x * s.flame.travel_x
                * constants::pi * s.flame.travel_y;
            const double prevBurnedVolume =
                lastTravel_x * lastTravel_x * constants::pi * lastTravel_y;
            const double litVolume = burnedVolume - prevBurnedVolume;
            const double n = (litVolume / p.volume) * s.system.n();

            const double fuelBurned =
                s.system.react(n * s.flame.efficiency, s.flame.globalMix);
            const double massFuelBurned = fuelBurned * p.fuelMass;
            s.system.changeEnergy(
                massFuelBurned * p.fuelEnergyDensity);

            s.flame.lit_n += n;
            s.flame.percentageLit += litVolume / p.volume;

            s.burntFuel += massFuelBurned;
        }
        else {
            s.lit = false;
        }

        s.flame.lastVolume = p.volume;
    }
}
ES_GAS_FUNCTION inline void advanceDistributed(View s,const Parameters &p,
    GasSystem &intake,GasSystem &exhaust,double dt) {
    begin(s,p,dt);
    GasSystem::FlowParameters flow{p.intakeK,dt,1,0,p.intakeArea,p.volume/p.cylinderHeight,&intake,&s.system};
    const double intakeFlow=GasSystem::flow(flow);
    s.system.dissipateExcessVelocity();
    flow.k_flow=p.exhaustK;
    flow.crossSectionArea_0=p.volume/p.cylinderHeight;
    flow.crossSectionArea_1=p.exhaustArea;
    flow.system_0=&s.system; flow.system_1=&exhaust;
    const double exhaustFlow=GasSystem::flow(flow);
    s.system.dissipateExcessVelocity();
    s.system.updateVelocity(dt,0.5);
    finish(s,p,intakeFlow,exhaustFlow,dt);
}
}
#endif
