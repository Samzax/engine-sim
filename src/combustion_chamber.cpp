#include "../include/combustion_chamber.h"

#include "../include/constants.h"
#include "../include/units.h"
#include "../include/piston.h"
#include "../include/connecting_rod.h"
#include "../include/utilities.h"
#include "../include/exhaust_system.h"
#include "../include/cylinder_bank.h"
#include "../include/engine.h"

#include <cmath>
#include <stdexcept>

CombustionChamber::CombustionChamber() {
    m_crankcasePressure = 0.0;
    m_piston = nullptr;
    m_head = nullptr;
    m_engine = nullptr;
    m_pistonSpeed = nullptr;
    m_pressure = nullptr;
    m_lit = false;
    m_litLastFrame = false;
    m_peakTemperature = 0;

    m_meanPistonSpeedToTurbulence = nullptr;
    m_nBurntFuel = 0;

    m_manifoldToRunnerFlowRate = 0;
    m_primaryToCollectorFlowRate = 0;
    m_cylinderWidthApproximation = 0;
    m_cylinderCrossSectionSurfaceArea = 0;

    m_lastTimestepTotalExhaustFlow = 0;
    m_lastTimestepTotalIntakeFlow = 0;
    m_exhaustFlow = 0;
    m_exhaustFlowRate = 0;
    m_intakeFlowRate = 0;

    m_fuel = nullptr;
}

CombustionChamber::~CombustionChamber() {
    assert(m_pistonSpeed == nullptr);
    assert(m_pressure == nullptr);
}

void CombustionChamber::initialize(const Parameters &params) {
    m_thermal.initialize(params.thermal);
    m_dynamicCombustion = params.dynamicCombustion;
    m_lubrication.initialize(params.lubrication);
    m_piston = params.Piston;
    m_head = params.Head;
    m_fuel = params.Fuel;
    m_crankcasePressure = params.CrankcasePressure;
    m_meanPistonSpeedToTurbulence = params.MeanPistonSpeedToTurbulence;

    m_pistonSpeed = new double[StateSamples];
    m_pressure = new double[StateSamples];
    m_pistonSpeedSum = 0;
    for (int i = 0; i < StateSamples; ++i) {
        m_pistonSpeed[i] = 0;
        m_pressure[i] = 0;
    }

    Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());

    m_manifoldToRunnerFlowRate = intake->getRunnerFlowRate();
    m_primaryToCollectorFlowRate = exhaust->getPrimaryFlowRate();

    const double bore_r = m_head->getCylinderBank()->getBore() / 2.0;
    m_cylinderCrossSectionSurfaceArea = constants::pi * bore_r * bore_r;
    m_cylinderWidthApproximation = std::sqrt(m_cylinderCrossSectionSurfaceArea);

    const double height = getVolume() / m_cylinderCrossSectionSurfaceArea;
    m_system.setGeometry(
        m_cylinderWidthApproximation,
        height,
        1.0,
        0.0);

    const double intakeRunnerCrossSection = m_head->getIntakeRunnerCrossSectionArea();
    const double intakeRunnerWidth = std::sqrt(intakeRunnerCrossSection);
    const double manifoldRunnerLength = intake->getRunnerLength();
    const double manifoldRunnerVolume = intakeRunnerCrossSection * manifoldRunnerLength;
    const double totalIntakeRunnerVolume = m_head->getIntakeRunnerVolume() + manifoldRunnerVolume;
    const double overallIntakeRunnerLength = totalIntakeRunnerVolume / intakeRunnerCrossSection;
    m_intakeRunnerAndManifold.initialize(
        units::pressure(1.0, units::atm),
        totalIntakeRunnerVolume,
        units::celcius(25.0));
    m_intakeRunnerAndManifold.setGeometry(
        overallIntakeRunnerLength,
        intakeRunnerWidth,
        1.0,
        0.0);

    const double exhaustRunnerCrossSection = m_head->getExhaustRunnerCrossSectionArea();
    const double exhaustRunnerWidth = std::sqrt(exhaustRunnerCrossSection);
    const double exhaustTubeLength =
        exhaust->getPrimaryTubeLength() + m_head->getHeaderPrimaryLength(m_piston->getCylinderIndex());
    const double exhaustTubeVolume = exhaustRunnerCrossSection * exhaustTubeLength;
    const double totalExhaustRunnerVolume = m_head->getExhaustRunnerVolume() + exhaustTubeVolume;
    const double overallExhaustRunnerLength = totalExhaustRunnerVolume / exhaustRunnerCrossSection;
    m_exhaustRunnerAndPrimary.initialize(
        units::pressure(1.0, units::atm),
        totalExhaustRunnerVolume,
        units::celcius(25.0));
    m_exhaustRunnerAndPrimary.setGeometry(
        overallExhaustRunnerLength,
        exhaustRunnerWidth,
        1.0,
        0.0);
    m_intakePipe.initialize(m_intakeRunnerAndManifold,overallIntakeRunnerLength,intakeRunnerCrossSection,params.pipeCells,params.pipeFrictionFactor);
    m_exhaustPipe.initialize(m_exhaustRunnerAndPrimary,overallExhaustRunnerLength,exhaustRunnerCrossSection,params.pipeCells,params.pipeFrictionFactor);
}

void CombustionChamber::destroy() {
    if (m_pistonSpeed != nullptr) delete[] m_pistonSpeed;
    if (m_pressure != nullptr) delete[] m_pressure;

    m_pistonSpeed = nullptr;
    m_pressure = nullptr;
}

double CombustionChamber::getVolume() const {
    const double combustionPortVolume = m_head->getCombustionChamberVolume();
    const CylinderBank *bank = m_head->getCylinderBank();

    const double area = bank->boreSurfaceArea();
    double pin_x, pin_y;
    m_piston->m_body.localToWorld(0, m_piston->getWristPinLocation(), &pin_x, &pin_y);
    const double s =
        (pin_x - bank->getX()) * bank->getDx()
        + (pin_y - bank->getY()) * bank->getDy();
    const double sweep =
        area * (bank->getDeckHeight() - s - m_piston->getCompressionHeight());

    return sweep + combustionPortVolume - m_piston->getDisplacement();
}

double CombustionChamber::pistonSpeed() const {
    const CylinderBank *bank = m_head->getCylinderBank();
    return
        m_piston->m_body.v_x * bank->getDx()
        + m_piston->m_body.v_y * bank->getDy();
}

double CombustionChamber::calculateMeanPistonSpeed() const {
    return m_pistonSpeedSum / StateSamples;
}

double CombustionChamber::calculateFiringPressure() const {
    double firingPressure = 0;
    for (int i = 0; i < StateSamples; ++i) {
        if (m_pressure[i] > firingPressure) {
            firingPressure = m_pressure[i];
        }
    }

    return firingPressure;
}

bool CombustionChamber::popLitLastFrame() {
    const bool lit = m_litLastFrame;
    m_litLastFrame = false;

    return lit;
}

void CombustionChamber::ignite() {
    if (!m_lit) {
        if (m_system.mix().p_fuel == 0) return;

        const double afr = m_system.mix().p_o2 / m_system.mix().p_fuel;
        const double equivalenceRatio = m_fuel->getMolecularAfr() / afr;
        if (equivalenceRatio < 0.5) return;
        else if (equivalenceRatio > 1.9) return;

        const double idealInert = m_system.mix().p_o2 / 0.7;
        const double dilution = (m_system.mix().p_inert / idealInert) - 1;

        m_flameEvent.lastVolume = getVolume();
        m_flameEvent.travel_x = 0;
        m_flameEvent.travel_y = 0;
        m_flameEvent.lit_n = 0;
        m_flameEvent.total_n = m_system.n();
        m_flameEvent.percentageLit = 0;
        m_flameEvent.globalMix = m_system.mix();
        m_flameEvent.ignitionTemperature = m_system.temperature();
        m_flameEvent.ignitionPressure = m_system.pressure();
        m_flameEvent.unburnedTemperature = m_system.temperature();
        m_lit = true;
        m_litLastFrame = true;

        const double randomness =
            m_fuel->getBurningEfficiencyRandomness();
        const double lowEfficiencyAttenuation =
            m_fuel->getLowEfficiencyAttenuation();
        const double maxBurningEfficiency =
            m_fuel->getMaxBurningEfficiency();
        const double maxTurbulenceEffect =
            m_fuel->getMaxTurbulenceEffect();
        const double maxDilutionEffect =
            m_fuel->getMaxDilutionEffect();

        const double turbulence =
            m_meanPistonSpeedToTurbulence->sampleTriangle(
                calculateMeanPistonSpeed());
        const double mixingFactor =
            1.0 - (
                clamp(turbulence / maxTurbulenceEffect)
                * clamp(1 - dilution / maxDilutionEffect));
        const double rand_s =
            lowEfficiencyAttenuation
            * ((1 - randomness) + randomness * ((double)rand() / RAND_MAX));
        const double efficiencyAttenuation =
            (mixingFactor * rand_s + (1 - mixingFactor));
        m_flameEvent.efficiency =
            efficiencyAttenuation * maxBurningEfficiency;
        m_flameEvent.flameSpeed = m_fuel->flameSpeed(
            turbulence,
            afr,
            m_system.temperature(),
            m_system.pressure(),
            calculateFiringPressure(),
            units::pressure(160, units::psi),
            m_dynamicCombustion ? m_flameEvent.globalMix.residualFraction : 0);
        if (!(m_flameEvent.flameSpeed > 0)) m_lit = false;
    }
}

void CombustionChamber::update(double dt) {
    m_system.setVolume(getVolume());

    updateCycleStates();
    m_lubrication.advance(dt, std::abs(getFrictionForce()*pistonSpeed()), m_thermal);
    if (m_dynamicCombustion && m_lit) {
        const auto &mix = m_flameEvent.globalMix;
        const double gamma = m_system.variableProperties()
            ? 1 + constants::R/GasSystem::mixtureCv(m_flameEvent.unburnedTemperature, mix)
            : m_system.heatCapacityRatio();
        // Approximate unburned-zone compression; using bulk burned-gas
        // temperature here would spuriously accelerate the remaining flame.
        m_flameEvent.unburnedTemperature = m_flameEvent.ignitionTemperature
            * std::pow(m_system.pressure()/m_flameEvent.ignitionPressure, (gamma-1)/gamma);
        m_flameEvent.flameSpeed = m_fuel->flameSpeed(
            m_meanPistonSpeedToTurbulence->sampleTriangle(calculateMeanPistonSpeed()),
            mix.p_o2/mix.p_fuel, m_flameEvent.unburnedTemperature,
            m_system.pressure(), 0, 0, mix.residualFraction);
    }

    m_intakeFlowRate = m_head->intakeFlowRate(m_piston->getCylinderIndex());
    m_exhaustFlowRate = m_head->exhaustFlowRate(m_piston->getCylinderIndex());
}

void CombustionChamber::flow(double dt) {
    int steps=0;
    while (dt>0) {
        if (++steps>10000) throw std::runtime_error("Cylinder pipe coupling requires excessive substeps; increase simulation frequency");
        double h=dt;
        if (m_intakePipe.active()) h=(std::min)(h,m_intakePipe.stableTimestep());
        if (m_exhaustPipe.active()) h=(std::min)(h,m_exhaustPipe.stableTimestep());
        flowStep(h);
        dt-=h;
    }
}

void CombustionChamber::flowIntakeReservoir(double dt) {
    Intake *intake=m_head->getIntake(m_piston->getCylinderIndex());
    GasSystem *endpoint=m_intakePipe.active() ? &m_intakePipe.first() : &m_intakeRunnerAndManifold;
    GasSystem::FlowParameters params{m_manifoldToRunnerFlowRate,dt,1,0,
        intake->getPlenumCrossSectionArea(),m_head->getIntakeRunnerCrossSectionArea(),
        &intake->m_system,endpoint};
    GasSystem::flow(params);
}

void CombustionChamber::flowExhaustReservoir(double dt) {
    ExhaustSystem *exhaust=m_head->getExhaustSystem(m_piston->getCylinderIndex());
    GasSystem *endpoint=m_exhaustPipe.active() ? &m_exhaustPipe.last() : &m_exhaustRunnerAndPrimary;
    GasSystem::FlowParameters params{m_primaryToCollectorFlowRate,dt,1,0,
        m_head->getExhaustRunnerCrossSectionArea(),exhaust->getCollectorCrossSectionArea(),
        endpoint,exhaust->getSystem()};
    GasSystem::flow(params);
}

void CombustionChamber::flowReservoirPorts(double dt) {
    if(!supportsSeparatedPorts()) throw std::logic_error("Separated ports require distributed intake and exhaust pipes");
    flowIntakeReservoir(dt);
    flowExhaustReservoir(dt);
}

void CombustionChamber::flowCylinderPorts(double dt) {
    if(!supportsSeparatedPorts()) throw std::logic_error("Separated ports require distributed intake and exhaust pipes");
    chamber_flow::advanceDistributed(cylinderFlowView(), cylinderFlowParameters(),
        m_intakePipe.last(), m_exhaustPipe.first(), dt);
}

chamber_flow::Parameters CombustionChamber::cylinderFlowParameters() const {
    const auto *bank=m_head->getCylinderBank();
    const double volume=getVolume();
    const double height=volume/m_cylinderCrossSectionSurfaceArea;
    return {volume,height,height*constants::pi*bank->getBore()+m_cylinderCrossSectionSurfaceArea*2,
        bank->getBore(),bank->boreSurfaceArea(),calculateMeanPistonSpeed(),
        m_piston->getBlowbyK(),m_crankcasePressure,m_intakeFlowRate,m_exhaustFlowRate,
        m_head->getIntakeRunnerCrossSectionArea(),m_head->getExhaustRunnerCrossSectionArea(),
        m_fuel->getMolecularMass(),m_fuel->getEnergyDensity()};
}

chamber_flow::View CombustionChamber::cylinderFlowView() {
    return {m_system,m_thermal,m_flameEvent,m_lit,m_peakTemperature,m_nBurntFuel,
        m_exhaustFlow,m_lastTimestepTotalExhaustFlow,m_lastTimestepTotalIntakeFlow};
}

chamber_flow::State CombustionChamber::cylinderFlowState() const {
    return {m_system,m_thermal,m_flameEvent,m_lit,m_peakTemperature,m_nBurntFuel,
        m_exhaustFlow,m_lastTimestepTotalExhaustFlow,m_lastTimestepTotalIntakeFlow};
}

void CombustionChamber::applyCylinderFlowState(const chamber_flow::State &state) {
    m_system=state.system; m_thermal=state.thermal; m_flameEvent=state.flame;
    m_lit=state.lit; m_peakTemperature=state.peakTemperature; m_nBurntFuel=state.burntFuel;
    m_exhaustFlow=state.exhaustFlow; m_lastTimestepTotalExhaustFlow=state.totalExhaustFlow;
    m_lastTimestepTotalIntakeFlow=state.totalIntakeFlow;
}

void CombustionChamber::flowStep(double dt, bool deferPipes, bool reservoirPortsDone) {
    const auto parameters = cylinderFlowParameters();
    const double volume = parameters.volume;
    const double cylinderHeight = parameters.cylinderHeight;
    chamber_flow::begin(cylinderFlowView(), parameters, dt);

    Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());
    GasSystem *intakeOut=m_intakePipe.active() ? &m_intakePipe.last() : &m_intakeRunnerAndManifold;
    GasSystem *exhaustIn=m_exhaustPipe.active() ? &m_exhaustPipe.first() : &m_exhaustRunnerAndPrimary;

    GasSystem::FlowParameters flowParams;
    flowParams.dt = dt;

    if (!reservoirPortsDone) flowIntakeReservoir(dt);

    if (!m_intakePipe.active()) m_intakeRunnerAndManifold.dissipateExcessVelocity();

    flowParams.k_flow = m_intakeFlowRate;
    flowParams.crossSectionArea_0 = m_head->getIntakeRunnerCrossSectionArea();
    flowParams.crossSectionArea_1 = volume / cylinderHeight;
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = intakeOut;
    flowParams.system_1 = &m_system;
    const double intakeFlow = GasSystem::flow(flowParams);

    if (!m_intakePipe.active()) m_intakeRunnerAndManifold.dissipateExcessVelocity();
    m_system.dissipateExcessVelocity();

    flowParams.k_flow = m_exhaustFlowRate;
    flowParams.crossSectionArea_0 = volume / cylinderHeight;
    flowParams.crossSectionArea_1 = m_head->getExhaustRunnerCrossSectionArea();
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &m_system;
    flowParams.system_1 = exhaustIn;
    const double exhaustFlow = GasSystem::flow(flowParams);

    m_system.dissipateExcessVelocity();
    if (!m_exhaustPipe.active()) m_exhaustRunnerAndPrimary.dissipateExcessVelocity();

    if (!reservoirPortsDone) flowExhaustReservoir(dt);

    if (!deferPipes) GasPipe::advancePair(m_intakePipe, m_exhaustPipe, dt);
    if (m_intakePipe.active()) {
        if (!deferPipes) m_intakePipe.aggregate(m_intakeRunnerAndManifold);
    } else m_intakeRunnerAndManifold.updateVelocity(dt, intake->getVelocityDecay());
    m_system.updateVelocity(dt, 0.5);
    if (m_exhaustPipe.active()) {
        if (!deferPipes) m_exhaustPipe.aggregate(m_exhaustRunnerAndPrimary);
    } else m_exhaustRunnerAndPrimary.updateVelocity(dt, exhaust->getVelocityDecay());

    chamber_flow::finish(cylinderFlowView(), parameters, intakeFlow, exhaustFlow, dt);
}

double CombustionChamber::lastEventAfr() const {
    const double totalFuel = m_flameEvent.globalMix.p_fuel * m_flameEvent.total_n;
    const double totalOxygen = m_flameEvent.globalMix.p_o2 * m_flameEvent.total_n;
    const double totalInert = m_flameEvent.globalMix.p_inert * m_flameEvent.total_n;

    const double fuelMolarMass = m_fuel->getMolecularMass();
    constexpr double oxygenMolarMass = units::mass(31.9988, units::g);
    constexpr double nitrogenMolarMass = units::mass(28.014, units::g);

    if (totalFuel == 0) return 0;
    else {
        return
            (oxygenMolarMass * totalOxygen + totalInert * nitrogenMolarMass)
            / (totalFuel * fuelMolarMass);
    }
}

double CombustionChamber::calculateFrictionForce(double v_s) const {
    const double cylinderWallForce = m_piston->calculateCylinderWallForce();

    const double F_coul = m_frictionModel.frictionCoeff * cylinderWallForce;
    const double v_st = m_frictionModel.breakawayFrictionVelocity * constants::root_2;
    const double v_coul = m_frictionModel.breakawayFrictionVelocity / 10;
    const double F_brk = m_frictionModel.breakawayFriction;
    const double v = std::abs(v_s);

    const double F_0 = constants::root_2 * constants::e * std::fmax(0.0, F_brk - F_coul);
    const double F_1 = v / v_st;
    const double F_2 = std::exp(-F_1 * F_1) * F_1;
    const double F_3 = F_coul * std::tanh(v / v_coul);
    const double F_4 = m_frictionModel.viscousFrictionCoefficient * v * m_lubrication.viscousMultiplier();

    return (F_0 * F_2 + F_3 + F_4) * m_lubrication.frictionScale();
}

void CombustionChamber::updateCycleStates() {
    double crankAngle = m_engine->getOutputCrankshaft()->getCycleAngle();
    if (std::isnan(crankAngle) || std::isinf(crankAngle)) {
        crankAngle = 0.0;
    }

    const int i = (int)std::round((crankAngle / (4 * constants::pi)) * (StateSamples - 1.0));

    const double speed = std::abs(pistonSpeed());
    m_pistonSpeedSum += speed - m_pistonSpeed[i];
    m_pistonSpeed[i] = speed;
    m_pressure[i] = m_system.pressure();
}

void CombustionChamber::apply(atg_scs::SystemState *system) {
    CylinderBank *bank = m_head->getCylinderBank();
    const double area = (bank->getBore() * bank->getBore() / 4.0) * constants::pi;
    const double v_x = system->v_x[m_piston->m_body.index];
    const double v_y = system->v_y[m_piston->m_body.index];

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    const double pressureDifferential = m_system.pressure() - m_crankcasePressure;
    const double force = -area * pressureDifferential;

    if (std::isnan(force) || std::isinf(force)) {
        assert(false);
    }

    constexpr double limit = 1E-3;
    const double abs_v_s = std::fmin(std::abs(v_s), limit);
    const double attenuation = abs_v_s / limit;

    const double F = calculateFrictionForce(v_s) * attenuation;
    const double F_fric = (v_s > 0)
        ? -F
        : F;

    system->applyForce(
        0.0,
        0.0,
        (force + F_fric) * bank->getDx(),
        (force + F_fric) * bank->getDy(),
        m_piston->m_body.index);
}

double CombustionChamber::getFrictionForce() const {
    CylinderBank *bank = m_head->getCylinderBank();
    const double v_x = m_piston->m_body.v_x;
    const double v_y = m_piston->m_body.v_y;

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    return calculateFrictionForce(v_s);
}
