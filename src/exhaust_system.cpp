#include "../include/exhaust_system.h"

#include "../include/units.h"

#include <cmath>
#include <stdexcept>

ExhaustSystem::ExhaustSystem() {
    m_primaryFlowRate = 0;
    m_outletFlowRate = 0;
    m_collectorCrossSectionArea = 0;
    m_length = 0;
    m_primaryTubeLength = 0;
    m_audioVolume = 0;
    m_velocityDecay = 0;
    m_flow = 0;
    m_index = -1;
    m_impulseResponse = nullptr;
}

ExhaustSystem::~ExhaustSystem() {
    /* void */
}

void ExhaustSystem::initialize(const Parameters &params) {
    m_templates.valid=false;
    const double volume = params.collectorCrossSectionArea * params.length;
    if (!std::isfinite(params.length) || params.length <= 0
        || !std::isfinite(params.collectorCrossSectionArea) || params.collectorCrossSectionArea <= 0
        || !std::isfinite(volume) || volume <= 0) {
        throw std::invalid_argument("Exhaust collector length, cross section area and volume must be finite and positive");
    }
    const double systemWidth = std::sqrt(params.collectorCrossSectionArea);
    const double systemLength = params.length;
    m_system.initialize(
            units::pressure(1.0, units::atm),
            volume,
            units::celcius(25.0));
    m_system.setGeometry(
        systemLength,
        systemWidth,
        1.0,
        0.0);

    m_atmosphere.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1000.0, units::m3),
        units::celcius(25.0));
    m_atmosphere.setGeometry(
        units::distance(10.0, units::m),
        units::distance(10.0, units::m),
        1.0,
        0.0);

    m_primaryFlowRate = params.primaryFlowRate;
    m_audioVolume = params.audioVolume;
    m_outletFlowRate = params.outletFlowRate;
    m_collectorCrossSectionArea = params.collectorCrossSectionArea;
    m_velocityDecay = params.velocityDecay;
    m_impulseResponse = params.impulseResponse;
    m_length = params.length;
    m_primaryTubeLength = params.primaryTubeLength;
}

void ExhaustSystem::destroy() {
    /* void */
}

reservoir_flow::ExhaustParameters ExhaustSystem::flowParameters() const {
    return {m_collectorCrossSectionArea,m_outletFlowRate,m_velocityDecay};
}
reservoir_flow::ExhaustState ExhaustSystem::flowState() const {
    reservoir_flow::prepare(m_templates,m_atmosphere,m_system.variableProperties());
    return {m_system,m_atmosphere,m_flow,m_templates};
}
void ExhaustSystem::applyFlowState(const reservoir_flow::ExhaustState &s) {
    m_system=s.system; m_atmosphere=s.atmosphere; m_flow=s.flow; m_templates=s.templates;
}
void ExhaustSystem::process(double dt) {
    reservoir_flow::process({m_system,m_atmosphere,m_flow,m_templates},flowParameters(),dt);
}
