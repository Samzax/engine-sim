#ifndef ENGINE_SIM_RESERVOIR_FLOW_H
#define ENGINE_SIM_RESERVOIR_FLOW_H
#include "gas_system.h"

namespace reservoir_flow {
struct IntakeParameters {
    double molecularAfr, fuelMass, oxygenPerFuel, throttlePlatePosition;
    double crossSectionArea, inputFlowK, idleFlowK, velocityDecay;
};
struct IntakeTemplates {
    GasSystem main, idle;
    double molecularAfr=0, fuelMass=0, oxygenPerFuel=0, volume=0;
    bool advanced=false, atmosphereAdvanced=false, valid=false;
};
struct ExhaustTemplate {
    GasSystem gas;
    double volume=0;
    bool advanced=false, atmosphereAdvanced=false, valid=false;
};
struct IntakeState {
    GasSystem system, atmosphere;
    double flow=0, totalFuelInjected=0, flowRate=0;
    IntakeTemplates templates;
};
struct IntakeView { GasSystem &system, &atmosphere; double &flow, &totalFuelInjected; IntakeTemplates &templates; };
ES_GAS_FUNCTION inline IntakeView view(IntakeState &s) { return {s.system,s.atmosphere,s.flow,s.totalFuelInjected,s.templates}; }
struct ExhaustParameters { double collectorArea, outletFlowRate, velocityDecay; };
struct ExhaustState { GasSystem system, atmosphere; double flow=0; ExhaustTemplate templates; };
struct ExhaustView { GasSystem &system, &atmosphere; double &flow; ExhaustTemplate &templates; };
ES_GAS_FUNCTION inline ExhaustView view(ExhaustState &s) { return {s.system,s.atmosphere,s.flow,s.templates}; }
// Preserve the finite atmosphere's initialized geometry and gas-property mode.
// Its imposed pressure, temperature and mixtures are constant between resets.
ES_GAS_FUNCTION inline void prepare(IntakeTemplates &cache,const GasSystem &atmosphere,
    bool advanced,const IntakeParameters &p) {
    if(cache.valid && cache.advanced==advanced && cache.atmosphereAdvanced==atmosphere.variableProperties()
        && cache.volume==atmosphere.volume() && cache.molecularAfr==p.molecularAfr
        && cache.fuelMass==p.fuelMass && cache.oxygenPerFuel==p.oxygenPerFuel) return;
    const double oxygenFraction = advanced ? 0.21 : 0.25;
    const double ideal_afr = advanced ? p.molecularAfr / oxygenFraction : 0.8 * p.molecularAfr * 4;

    const double p_air = ideal_afr / (1 + ideal_afr);
    GasSystem::Mix fuelAirMix;
    fuelAirMix.fuelMolecularMass = p.fuelMass;
    fuelAirMix.oxygenPerFuel = p.oxygenPerFuel;
    fuelAirMix.p_fuel = 1 - p_air;
    fuelAirMix.p_inert = p_air * (1-oxygenFraction);
    fuelAirMix.p_o2 = p_air * oxygenFraction;

    const double idle_afr = 2.0;
    const double p_idle_air = idle_afr / (1 + idle_afr);
    GasSystem::Mix fuelMix;
    fuelMix.fuelMolecularMass = p.fuelMass;
    fuelMix.oxygenPerFuel = p.oxygenPerFuel;
    fuelMix.p_fuel = (1.0 - p_idle_air);
    fuelMix.p_inert = p_idle_air * (1-oxygenFraction);
    fuelMix.p_o2 = p_idle_air * oxygenFraction;

    cache.main=atmosphere; cache.idle=atmosphere;
    cache.main.reset(units::pressure(1.0,units::atm),units::celcius(25.0),fuelAirMix);
    cache.idle.reset(units::pressure(1.0,units::atm),units::celcius(25.0),fuelMix);
    cache.main.temperature(); cache.idle.temperature();
    cache.main.primeFlowConstants(); cache.idle.primeFlowConstants();
    cache.advanced=advanced; cache.atmosphereAdvanced=atmosphere.variableProperties();
    cache.volume=atmosphere.volume(); cache.molecularAfr=p.molecularAfr;
    cache.fuelMass=p.fuelMass; cache.oxygenPerFuel=p.oxygenPerFuel; cache.valid=true;
}
ES_GAS_FUNCTION inline void process(IntakeView v,const IntakeParameters &p,double dt) {
    prepare(v.templates,v.atmosphere,v.system.variableProperties(),p);
    const double throttle = p.throttlePlatePosition;
    const double flowAttenuation = std::cos(throttle * constants::pi / 2);

    GasSystem::FlowParameters flowParams;
    flowParams.crossSectionArea_0 = units::area(10, units::m2);
    flowParams.crossSectionArea_1 = p.crossSectionArea;
    flowParams.direction_x = 0.0;
    flowParams.direction_y = -1.0;
    flowParams.dt = dt;

    v.atmosphere=v.templates.main;
    flowParams.system_0 = &v.atmosphere;
    flowParams.system_1 = &v.system;
    flowParams.k_flow = flowAttenuation * p.inputFlowK;
    v.flow = v.system.flow(flowParams);

    v.atmosphere=v.templates.idle;
    flowParams.system_0 = &v.atmosphere;
    flowParams.system_1 = &v.system;
    flowParams.k_flow = p.idleFlowK;
    const double idleCircuitFlow = v.system.flow(flowParams);

    v.system.dissipateExcessVelocity();
    v.system.updateVelocity(dt, p.velocityDecay);

    if (v.flow > 0) {
        v.totalFuelInjected += v.templates.main.mix().p_fuel * v.flow;
    }

    if (idleCircuitFlow > 0) {
        v.totalFuelInjected += v.templates.idle.mix().p_fuel * idleCircuitFlow;
    }
}

ES_GAS_FUNCTION inline void prepare(ExhaustTemplate &cache,const GasSystem &atmosphere,bool advanced) {
    if(cache.valid && cache.advanced==advanced && cache.atmosphereAdvanced==atmosphere.variableProperties()
        && cache.volume==atmosphere.volume()) return;
    GasSystem::Mix airMix;
    airMix.p_fuel = 0;
    airMix.p_inert = 1.0;
    airMix.p_o2 = 0.0;
    if (advanced) {
        airMix.p_inert = 0.79;
        airMix.p_o2 = 0.21;
    }

    cache.gas=atmosphere;
    cache.gas.reset(units::pressure(1.0, units::atm), units::celcius(25.0), airMix);
    cache.gas.temperature(); cache.gas.primeFlowConstants(); cache.advanced=advanced;
    cache.atmosphereAdvanced=atmosphere.variableProperties(); cache.volume=atmosphere.volume(); cache.valid=true;
}
ES_GAS_FUNCTION inline void process(ExhaustView v,const ExhaustParameters &p,double dt) {
    prepare(v.templates,v.atmosphere,v.system.variableProperties());
    v.atmosphere=v.templates.gas;
    GasSystem::FlowParameters flowParams;
    flowParams.crossSectionArea_0 = p.collectorArea;
    flowParams.crossSectionArea_1 = units::area(10, units::m2);
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.dt = dt;
    flowParams.system_0 = &v.atmosphere;
    flowParams.system_1 = &v.system;
    flowParams.k_flow = p.outletFlowRate;

    v.flow = v.system.flow(flowParams);

    v.system.dissipateExcessVelocity();
    v.system.updateVelocity(dt, p.velocityDecay);
}

}
#endif
