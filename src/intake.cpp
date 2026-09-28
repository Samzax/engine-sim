#include "../include/intake.h"

#include "../include/units.h"

#include <cmath>
#include <stdexcept>

Intake::Intake() {
    m_inputFlowK = 0;
    m_idleFlowK = 0;
    m_flow = 0;
    m_throttle = 1.0;
    m_idleThrottlePlatePosition = 0.0;
    m_crossSectionArea = 0.0;
    m_flowRate = 0;
    m_totalFuelInjected = 0;
    m_molecularAfr = 0;
    m_runnerLength = 0;
}

Intake::~Intake() {
    /* void */
}

void Intake::initialize(Parameters &params) {
    m_templates.valid=false;
    const double length = params.volume / params.CrossSectionArea;
    if (!std::isfinite(params.volume) || params.volume <= 0
        || !std::isfinite(params.CrossSectionArea) || params.CrossSectionArea <= 0
        || !std::isfinite(length) || length <= 0) {
        throw std::invalid_argument("Intake plenum volume, cross section area and length must be finite and positive");
    }
    const double width = std::sqrt(params.CrossSectionArea);
    m_system.initialize(
        units::pressure(1.0, units::atm),
        params.volume,
        units::celcius(25.0));
    m_system.setGeometry(
        width,
        length,
        1.0,
        0.0);

    m_atmosphere.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1000.0, units::m3),
        units::celcius(25.0));
    m_atmosphere.setGeometry(
        units::distance(100.0, units::m),
        units::distance(100.0, units::m),
        1.0,
        0.0);

    m_inputFlowK = params.InputFlowK;
    m_molecularAfr = params.MolecularAfr;
    m_idleFlowK = params.IdleFlowK;
    m_idleThrottlePlatePosition = params.IdleThrottlePlatePosition;
    m_runnerLength = params.RunnerLength;
    m_crossSectionArea = params.CrossSectionArea;
    m_velocityDecay = params.VelocityDecay;
    m_runnerFlowRate = params.RunnerFlowRate;
}

void Intake::destroy() {
    /* void */
}

reservoir_flow::IntakeParameters Intake::flowParameters() const {
    return {m_molecularAfr,m_fuelMass,m_oxygenPerFuel,getThrottlePlatePosition(),
        m_crossSectionArea,m_inputFlowK,m_idleFlowK,m_velocityDecay};
}
reservoir_flow::IntakeState Intake::flowState() const {
    reservoir_flow::prepare(m_templates,m_atmosphere,m_system.variableProperties(),flowParameters());
    return {m_system,m_atmosphere,m_flow,m_totalFuelInjected,m_flowRate,m_templates};
}
void Intake::applyFlowState(const reservoir_flow::IntakeState &s) {
    m_system=s.system; m_atmosphere=s.atmosphere; m_flow=s.flow;
    m_totalFuelInjected=s.totalFuelInjected; m_flowRate=s.flowRate; m_templates=s.templates;
}
void Intake::process(double dt) {
    reservoir_flow::process({m_system,m_atmosphere,m_flow,m_totalFuelInjected,m_templates},flowParameters(),dt);
}
