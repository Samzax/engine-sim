#include "../include/gas_system.h"

#include "../include/units.h"
#include "../include/utilities.h"
#include "../include/gas_thermo.h"

#include <cmath>
#include <cassert>

double GasSystem::molecularMass(const Mix &m) {
    return (m.p_inert - m.p_co2 - m.p_h2o) * 0.028014 + m.p_o2 * 0.0319988
        + m.p_co2 * 0.0440098 + m.p_h2o * 0.0180154 + m.p_fuel * m.fuelMolecularMass;
}

double GasSystem::mixtureEnergy(double t, const Mix &m) {
    return (m.p_inert-m.p_co2-m.p_h2o)*gas_thermo::u(0,t) + m.p_o2*gas_thermo::u(1,t)
        + m.p_co2*gas_thermo::u(2,t) + m.p_h2o*gas_thermo::u(3,t) + m.p_fuel*gas_thermo::u(4,t);
}

double GasSystem::mixtureCv(double t, const Mix &m) {
    return (m.p_inert-m.p_co2-m.p_h2o)*gas_thermo::cv(0,t) + m.p_o2*gas_thermo::cv(1,t)
        + m.p_co2*gas_thermo::cv(2,t) + m.p_h2o*gas_thermo::cv(3,t) + m.p_fuel*gas_thermo::cv(4,t);
}

void GasSystem::refreshProperties() const {
    if (m_propertiesValid) return;
    const Mix &m = m_state.mix;
    const double weights[5] = {m.p_inert-m.p_co2-m.p_h2o, m.p_o2, m.p_co2, m.p_h2o, m.p_fuel};
    for (int k=0; k<5; ++k) {
        m_lowCp[k] = m_highCp[k] = 0;
        for (int s=0; s<5; ++s) {
            m_lowCp[k] += weights[s]*gas_thermo::coefficients[s][0][k];
            m_highCp[k] += weights[s]*gas_thermo::coefficients[s][1][k];
        }
    }
    m_cv200 = gas_thermo::cvPolynomial(m_lowCp, 200);
    m_cv6000 = gas_thermo::cvPolynomial(m_highCp, 6000);
    m_u200 = 200*m_cv200;
    m_u1000 = m_u200 + gas_thermo::integral(m_lowCp,1000)-gas_thermo::integral(m_lowCp,200);
    m_u6000 = m_u1000 + gas_thermo::integral(m_highCp,6000)-gas_thermo::integral(m_highCp,1000);
    refreshMass();
    m_propertiesValid = true;
}

double GasSystem::molarEnergy(double t) const {
    refreshProperties();
    if (t <= 200) return m_cv200*t;
    if (t <= 1000) return m_u200+gas_thermo::integral(m_lowCp,t)-gas_thermo::integral(m_lowCp,200);
    if (t <= 6000) return m_u1000+gas_thermo::integral(m_highCp,t)-gas_thermo::integral(m_highCp,1000);
    return m_u6000 + m_cv6000*(t-6000);
}

double GasSystem::molarCv(double t) const {
    refreshProperties();
    if (t <= 200) return m_cv200;
    if (t >= 6000) return m_cv6000;
    return gas_thermo::cvPolynomial(t<=1000 ? m_lowCp : m_highCp, t);
}

double GasSystem::energyAtTemperature(double t) const {
    return n() * (m_variableProperties ? molarEnergy(t)
        : kineticEnergyPerMol(t, m_degreesOfFreedom));
}

double GasSystem::heatCapacity(double t) const {
    return n() * (m_variableProperties ? molarCv(t)
        : 0.5 * m_degreesOfFreedom * constants::R);
}

void GasSystem::setVariableProperties(bool enabled) {
    const double t = temperature();
    m_variableProperties = enabled;
    invalidateProperties();
    m_state.E_k = energyAtTemperature(t);
    m_cachedEnergy = -1;
}

double GasSystem::temperature() const {
    if (n() <= 0) return 0;
    if (!m_variableProperties) return kineticEnergy() / heatCapacity(300);
    if (m_cachedEnergy == kineticEnergy() && m_cachedN == n()) return m_cachedTemperature;
    const double target = (std::max)(0.0, kineticEnergy() / n());
    double lower = 0, upper = (std::max)(6000.0, target / constants::R);
    double t = std::clamp(m_cachedTemperature, lower, upper);
    for (int i = 0; i < 32; ++i) {
        const double residual = molarEnergy(t) - target;
        if (std::abs(residual) <= 1e-10 * (std::max)(1.0, target)) break;
        if (residual > 0) upper = t; else lower = t;
        const double next = t - residual / molarCv(t);
        t = next > lower && next < upper ? next : 0.5 * (lower + upper);
    }
    m_cachedEnergy = kineticEnergy(); m_cachedN = n(); m_cachedTemperature = t;
    return t;
}

void GasSystem::setGeometry(double width, double height, double dx, double dy) {
    m_width = width;
    m_height = height;
    m_dx = dx;
    m_dy = dy;
}

void GasSystem::initialize(double P, double V, double T, const Mix &mix, int degreesOfFreedom) {
    m_degreesOfFreedom = degreesOfFreedom;
    m_state.n_mol = P * V / (constants::R * T);
    m_state.V = V;
    m_state.E_k = T * (0.5 * degreesOfFreedom * m_state.n_mol * constants::R);
    m_state.mix = mix;
    invalidateProperties();
    if (m_variableProperties) m_state.E_k = energyAtTemperature(T);
    m_cachedEnergy = -1;
    m_state.momentum[0] = m_state.momentum[1] = 0;

    const double hcr = heatCapacityRatio();
    m_chokedFlowLimit = chokedFlowLimit(degreesOfFreedom);
    m_chokedFlowFactorCached = chokedFlowRate(degreesOfFreedom);
}

void GasSystem::reset(double P, double T, const Mix &mix) {
    m_state.n_mol = P * volume() / (constants::R * T);
    m_state.E_k = T * (0.5 * m_degreesOfFreedom * m_state.n_mol * constants::R);
    m_state.mix = mix;
    invalidateProperties();
    if (m_variableProperties) m_state.E_k = energyAtTemperature(T);
    m_cachedEnergy = -1;
    m_state.momentum[0] = m_state.momentum[1] = 0;
}

void GasSystem::setVolume(double V) {
    return changeVolume(V - m_state.V);
}

void GasSystem::setN(double n) {
    m_state.E_k = kineticEnergy(n);
    m_state.n_mol = n;
}

void GasSystem::changeVolume(double dV) {
    const double V = this->volume();
    const double L = std::pow(V + dV, 1 / 3.0);
    const double surfaceArea = (L * L);
    const double dL = -dV / surfaceArea;
    const double W = dL * pressure() * surfaceArea;

    m_state.V += dV;
    m_state.E_k += W;
}

void GasSystem::changePressure(double dP) {
    if (m_variableProperties) {
        if (n() > 0) m_state.E_k = energyAtTemperature((std::max)(0.0, temperature() + dP * volume() / (n() * constants::R)));
        return;
    }
    m_state.E_k += dP * volume() * m_degreesOfFreedom * 0.5;
}

void GasSystem::changeTemperature(double dT) {
    if (m_variableProperties) {
        m_state.E_k = energyAtTemperature((std::max)(0.0, temperature() + dT));
        return;
    }
    m_state.E_k += dT * 0.5 * m_degreesOfFreedom * n() * constants::R;
}

void GasSystem::changeEnergy(double dE) {
    m_state.E_k += dE;
}

void GasSystem::changeMix(const Mix &mix) {
    m_state.mix = mix;
    invalidateProperties();
    m_cachedEnergy = -1;
}

void GasSystem::injectFuel(double n) {
    if (m_variableProperties) {
        Mix fuel = m_state.mix;
        fuel.p_fuel=1; fuel.p_inert=fuel.p_o2=fuel.p_co2=fuel.p_h2o=0;
        gainN(n, mixtureEnergy(temperature(), fuel), fuel);
        return;
    }
    const double n_fuel = this->n_fuel() + n;
    const double p_fuel = n_fuel / this->n();
    m_state.mix.p_fuel = p_fuel;
}

void GasSystem::changeTemperature(double dT, double n) {
    if (m_variableProperties) {
        const double t = temperature();
        m_state.E_k += n * (mixtureEnergy((std::max)(0.0, t+dT), m_state.mix) - mixtureEnergy(t, m_state.mix));
        return;
    }
    m_state.E_k += dT * 0.5 * m_degreesOfFreedom * n * constants::R;
}

double GasSystem::react(double n, const Mix &mix) {
    if (m_variableProperties) {
        const double s = mix.oxygenPerFuel;
        // Equivalent hydrocarbon C_x H_y from molecular mass and oxygen demand.
        const double carbon = (mix.fuelMolecularMass - 4 * 0.001008 * s) / (0.012011 - 4 * 0.001008);
        const double water = 2 * (s - carbon);
        if (!(s > 0) || carbon < 0 || water < 0) return 0;
        const double burned = (std::max)(0.0, (std::min)({n_fuel(), n * mix.p_fuel,
            n_o2() / s, n * mix.p_o2 / s}));
        if (burned == 0) return 0;
        const double oldN = this->n();
        const double oldReference = energyAtTemperature(298.15);
        const Mix old = m_state.mix;
        m_state.mix.residualFraction = old.residualFraction + (1-old.residualFraction)*burned/(old.p_fuel*oldN);
        const double newN = oldN + burned * (carbon + water - 1 - s);
        m_state.mix.p_fuel = (old.p_fuel * oldN - burned) / newN;
        m_state.mix.p_o2 = (old.p_o2 * oldN - s * burned) / newN;
        m_state.mix.p_co2 = (old.p_co2 * oldN + carbon * burned) / newN;
        m_state.mix.p_h2o = (old.p_h2o * oldN + water * burned) / newN;
        m_state.mix.p_inert = (old.p_inert * oldN + (carbon + water) * burned) / newN;
        m_state.n_mol = newN;
        invalidateProperties();
        m_cachedEnergy = -1;
        // Fuel energy density supplies the reaction enthalpy at 298.15 K.
        // Convert it to internal energy and maintain our sensible-energy datum.
        m_state.E_k += energyAtTemperature(298.15) - oldReference
            + (newN - oldN) * constants::R * 298.15;
        return burned;
    }
    const double l_n_fuel = mix.p_fuel * n;
    const double l_n_o2 = mix.p_o2 * n;

    const double system_n_fuel = n_fuel();
    const double system_n_o2 = n_o2();
    const double system_n_inert = n_inert();
    const double system_n = this->n();

    // Assuming the following reaction:
    // 25[O2] + 2[C8H16] -> 16[CO2] + 18[H2O]
    constexpr double ideal_o2_ratio = 25.0 / 2;
    constexpr double ideal_fuel_ratio = 2.0 / 25;
    constexpr double output_input_ratio = (16.0 + 18.0) / (25 + 2);

    const double ideal_fuel_n = ideal_fuel_ratio * l_n_o2;
    const double ideal_o2_n = ideal_o2_ratio * l_n_fuel;
    
    const double a_n_fuel = std::fmin(
        std::fmin(system_n_fuel, l_n_fuel),
        ideal_fuel_n);
    const double a_n_o2 = std::fmin(
        std::fmin(system_n_o2, l_n_o2),
        ideal_o2_n);

    const double reactants_n = a_n_fuel + a_n_o2;
    const double products_n = output_input_ratio * reactants_n;
    const double dn = products_n - reactants_n;

    m_state.n_mol += dn;

    // Adjust mix
    const double new_system_n_fuel = system_n_fuel - a_n_fuel;
    const double new_system_n_o2 = system_n_o2 - a_n_o2;
    const double new_system_n_inert = system_n_inert + products_n;
    const double new_system_n = system_n + dn;

    if (new_system_n != 0) {
        m_state.mix.p_fuel = new_system_n_fuel / new_system_n;
        m_state.mix.p_inert = new_system_n_inert / new_system_n;
        m_state.mix.p_o2 = new_system_n_o2 / new_system_n;
    }
    else {
        m_state.mix.p_fuel = m_state.mix.p_inert = m_state.mix.p_o2 = 0;
    }

    return a_n_fuel;
}

double GasSystem::flowConstant(
    double targetFlowRate,
    double P,
    double pressureDrop,
    double T,
    double hcr)
{
    const double T_0 = T;
    const double p_0 = P, p_T = P - pressureDrop; // p_0 = upstream pressure

    const double chokedFlowLimit =
        std::pow((2.0 / (hcr + 1)), hcr / (hcr - 1));
    const double p_ratio = p_T / p_0;

    double flowRate = 0;
    if (p_ratio <= chokedFlowLimit) {
        // Choked flow
        flowRate = std::sqrt(hcr);
        flowRate *= std::pow(2 / (hcr + 1), (hcr + 1) / (2 * (hcr - 1)));
    }
    else {
        flowRate = (2 * hcr) / (hcr - 1);
        flowRate *= (1 - std::pow(p_ratio, (hcr - 1) / hcr));
        flowRate = std::sqrt(flowRate);
        flowRate *= std::pow(p_ratio, 1 / hcr);
    }

    flowRate *= p_0 / std::sqrt(constants::R * T_0);

    return targetFlowRate / flowRate;
}

double GasSystem::k_28inH2O(double flowRateScfm) {
    return flowConstant(
        units::flow(flowRateScfm, units::scfm),
        units::pressure(1.0, units::atm),
        units::pressure(28.0, units::inH2O),
        units::celcius(25),
        heatCapacityRatio(5)
    );
}

double GasSystem::k_carb(double flowRateScfm) {
    return flowConstant(
        units::flow(flowRateScfm, units::scfm),
        units::pressure(1.0, units::atm),
        units::pressure(1.5, units::inHg),
        units::celcius(25),
        heatCapacityRatio(5)
    );
}

double GasSystem::flowRate(
    double k_flow,
    double P0,
    double P1,
    double T0,
    double T1,
    double hcr,
    double chokedFlowLimit,
    double chokedFlowRateCached)
{
    if (k_flow == 0) return 0;

    double direction;
    double T_0;
    double p_0, p_T; // p_0 = upstream pressure
    if (P0 > P1) {
        direction = 1.0;
        T_0 = T0;
        p_0 = P0;
        p_T = P1;
    }
    else {
        direction = -1.0;
        T_0 = T1;
        p_0 = P1;
        p_T = P0;
    }

    const double p_ratio = p_T / p_0;
    double flowRate = 0;
    if (p_ratio <= chokedFlowLimit) {
        // Choked flow
        flowRate = chokedFlowRateCached;
        flowRate /= std::sqrt(constants::R * T_0);
    }
    else {
        const double s = std::pow(p_ratio, 1 / hcr);

        flowRate = (2 * hcr) / (hcr - 1);
        flowRate *= s * (s - p_ratio);
        flowRate = std::sqrt(std::fmax(flowRate, 0.0) / (constants::R * T_0));
    }

    flowRate *= direction * p_0;

    return flowRate * k_flow;
}

double GasSystem::loseN(double dn, double E_k_per_mol) {
    m_state.E_k -= E_k_per_mol * dn;
    m_state.n_mol -= dn;

    if (m_state.n_mol < 0) {
        m_state.n_mol = 0;
    }

    return dn;
}

double GasSystem::gainN(double dn, double E_k_per_mol, const Mix &mix) {
    const double next_n = m_state.n_mol + dn;
    const double current_n = m_state.n_mol;

    m_state.E_k += dn * E_k_per_mol;
    m_state.n_mol = next_n;

    if (next_n != 0) {
        const double incomingMass = dn*molecularMass(mix);
        const double existingMass = current_n*molecularMass(m_state.mix);
        if (incomingMass+existingMass > 0)
            m_state.mix.residualFraction = (existingMass*m_state.mix.residualFraction
                + incomingMass*mix.residualFraction)/(incomingMass+existingMass);
        const double fuelN = m_state.mix.p_fuel * current_n + dn * mix.p_fuel;
        if (fuelN > 0) {
            m_state.mix.fuelMolecularMass = (m_state.mix.fuelMolecularMass * m_state.mix.p_fuel * current_n
                + dn * mix.p_fuel * mix.fuelMolecularMass) / fuelN;
            m_state.mix.oxygenPerFuel = (m_state.mix.oxygenPerFuel * m_state.mix.p_fuel * current_n
                + dn * mix.p_fuel * mix.oxygenPerFuel) / fuelN;
        }
        m_state.mix.p_co2 = (m_state.mix.p_co2 * current_n + dn * mix.p_co2) / next_n;
        m_state.mix.p_h2o = (m_state.mix.p_h2o * current_n + dn * mix.p_h2o) / next_n;
        m_state.mix.p_fuel = (m_state.mix.p_fuel * current_n + dn * mix.p_fuel) / next_n;
        m_state.mix.p_inert = (m_state.mix.p_inert * current_n + dn * mix.p_inert) / next_n;
        m_state.mix.p_o2 = (m_state.mix.p_o2 * current_n + dn * mix.p_o2) / next_n;
    }
    else {
        m_state.mix.p_fuel = m_state.mix.p_inert = m_state.mix.p_o2 = 0;
        m_state.mix.p_co2 = m_state.mix.p_h2o = 0;
    }
    m_cachedEnergy = -1;
    invalidateProperties();

    return -dn;
}

void GasSystem::dissipateExcessVelocity() {
    const double v_x = velocity_x();
    const double v_y = velocity_y();
    const double v_squared = v_x * v_x + v_y * v_y;
    const double c = this->c();
    const double c_squared = c * c;

    if (c_squared >= v_squared || v_squared == 0) {
        return;
    }

    const double k_squared = c_squared / v_squared;
    const double k = std::sqrt(k_squared);

    m_state.momentum[0] *= k;
    m_state.momentum[1] *= k;

    m_state.E_k += 0.5 * mass() * (v_squared - c_squared);

    if (m_state.E_k < 0) m_state.E_k = 0;
}

void GasSystem::updateVelocity(double dt, double beta) {
    if (n() == 0) return;

    const double depth = volume() / (m_width * m_height);
    
    double d_momentum_x = 0;
    double d_momentum_y = 0;

    const double p0 = dynamicPressure(m_dx, m_dy);
    const double p1 = dynamicPressure(-m_dx, -m_dy);
    const double p2 = dynamicPressure(m_dy, m_dx);
    const double p3 = dynamicPressure(-m_dy, -m_dx);

    const double p_sa_0 = p0 * (m_height * depth);
    const double p_sa_1 = p1 * (m_height * depth);
    const double p_sa_2 = p2 * (m_width * depth);
    const double p_sa_3 = p3 * (m_width * depth);

    d_momentum_x += p_sa_0 * m_dx;
    d_momentum_y += p_sa_0 * m_dy;

    d_momentum_x -= p_sa_1 * m_dx;
    d_momentum_y -= p_sa_1 * m_dy;

    d_momentum_x += p_sa_2 * m_dy;
    d_momentum_y += p_sa_2 * m_dx;

    d_momentum_x -= p_sa_3 * m_dy;
    d_momentum_y -= p_sa_3 * m_dx;

    const double m = mass();
    const double inv_m = 1 / m;
    const double v0_x = m_state.momentum[0] * inv_m;
    const double v0_y = m_state.momentum[1] * inv_m;

    m_state.momentum[0] -= d_momentum_x * dt * beta;
    m_state.momentum[1] -= d_momentum_y * dt * beta;

    const double v1_x = m_state.momentum[0] * inv_m;
    const double v1_y = m_state.momentum[1] * inv_m;

    m_state.E_k -= 0.5 * m * (v1_x * v1_x - v0_x * v0_x);
    m_state.E_k -= 0.5 * m * (v1_y * v1_y - v0_y * v0_y);

    if (m_state.E_k < 0) m_state.E_k = 0;
}

void GasSystem::dissipateVelocity(double dt, double timeConstant) {
    if (n() == 0) return;

    const double invMass = 1.0 / mass();
    const double velocity_x = m_state.momentum[0] * invMass;
    const double velocity_y = m_state.momentum[1] * invMass;
    const double velocity_squared =
        velocity_x * velocity_x + velocity_y * velocity_y;

    const double s = dt / (dt + timeConstant);
    m_state.momentum[0] = m_state.momentum[0] * (1 - s);
    m_state.momentum[1] = m_state.momentum[1] * (1 - s);

    const double newVelocity_x = m_state.momentum[0] * invMass;
    const double newVelocity_y = m_state.momentum[1] * invMass;
    const double newVelocity_squared =
        newVelocity_x * newVelocity_x + newVelocity_y * newVelocity_y;

    const double dE_k = 0.5 * mass() * (velocity_squared - newVelocity_squared);
    m_state.E_k += dE_k;
}

double GasSystem::flow(const FlowParameters &params) {
    GasSystem *source = nullptr, *sink = nullptr;
    double sourcePressure = 0, sinkPressure = 0;
    double dx, dy;
    double sourceCrossSection = 0, sinkCrossSection = 0;
    double direction = 0;

    const double P_0 =
        params.system_0->pressure()
        + params.system_0->dynamicPressure(params.direction_x, params.direction_y);
    const double P_1 =
        params.system_1->pressure()
        + params.system_1->dynamicPressure(-params.direction_x, -params.direction_y);

    if (P_0 > P_1) {
        dx = params.direction_x;
        dy = params.direction_y;
        source = params.system_0;
        sink = params.system_1;
        sourcePressure = P_0;
        sinkPressure = P_1;
        sourceCrossSection = params.crossSectionArea_0;
        sinkCrossSection = params.crossSectionArea_1;
        direction = 1.0;
    }
    else {
        dx = -params.direction_x;
        dy = -params.direction_y;
        source = params.system_1;
        sink = params.system_0;
        sourcePressure = P_1;
        sinkPressure = P_0;
        sourceCrossSection = params.crossSectionArea_1;
        sinkCrossSection = params.crossSectionArea_0;
        direction = -1.0;
    }

    if (params.dt <= 0 || source->n() <= 0) return 0;
    const double gamma = source->heatCapacityRatio();
    double flow = params.dt * flowRate(
        params.k_flow,
        sourcePressure,
        sinkPressure,
        source->temperature(),
        sink->temperature(),
        gamma,
        source->m_variableProperties ? std::pow(2/(gamma+1), gamma/(gamma-1)) : source->m_chokedFlowLimit,
        source->m_variableProperties ? std::sqrt(gamma)*std::pow(2/(gamma+1), (gamma+1)/(2*(gamma-1))) : source->m_chokedFlowFactorCached);
    if (source->m_variableProperties)
        flow *= std::sqrt(units::AirMolecularMass / molecularMass(source->mix()));
    flow = clamp(flow, 0.0, 0.9 * source->n());

    const double fraction = flow / source->n();
    const double fractionVolume = fraction * source->volume();
    const double fractionMass = fraction * source->mass();
    const double remainingMass = (1 - fraction) * source->mass();

    if (flow != 0) {
        // - Stage 1
        // Fraction flows from source to sink.

        const double E_k_bulk_src0 = source->bulkKineticEnergy();
        const double E_k_bulk_sink0 = sink->bulkKineticEnergy();

        const double s0 = source->totalEnergy() + sink->totalEnergy();

        const double E_k_per_mol = source->kineticEnergyPerMol();
        sink->gainN(flow, E_k_per_mol, source->mix());
        source->loseN(flow, E_k_per_mol);

        const double s1 = source->totalEnergy() + sink->totalEnergy();

        const double dp_x = source->m_state.momentum[0] * fraction;
        const double dp_y = source->m_state.momentum[1] * fraction;
        source->m_state.momentum[0] -= dp_x;
        source->m_state.momentum[1] -= dp_y;

        sink->m_state.momentum[0] += dp_x;
        sink->m_state.momentum[1] += dp_y;

        const double E_k_bulk_src1 = source->bulkKineticEnergy();
        const double E_k_bulk_sink1 = sink->bulkKineticEnergy();

        sink->m_state.E_k -= ((E_k_bulk_src1 + E_k_bulk_sink1) - (E_k_bulk_src0 + E_k_bulk_sink0));
    }
    
    const double sourceMass = source->mass();
    const double invSourceMass = 1 / sourceMass;
    const double sinkMass = sink->mass();
    const double invSinkMass = 1 / sinkMass;

    const double c_source = source->c();
    const double c_sink = sink->c();

    const double sourceInitialMomentum_x = source->m_state.momentum[0];
    const double sourceInitialMomentum_y = source->m_state.momentum[1];

    const double sinkInitialMomentum_x = sink->m_state.momentum[0];
    const double sinkInitialMomentum_y = sink->m_state.momentum[1];

    // Momentum in fraction

    if (sinkCrossSection != 0) {
        const double sinkFractionVelocity =
            clamp((fractionVolume / sinkCrossSection) / params.dt, 0.0, c_sink);
        const double sinkFractionVelocity_squared = sinkFractionVelocity * sinkFractionVelocity;
        const double sinkFractionVelocity_x = sinkFractionVelocity * dx;
        const double sinkFractionVelocity_y = sinkFractionVelocity * dy;
        const double sinkFractionMomentum_x = sinkFractionVelocity_x * fractionMass;
        const double sinkFractionMomentum_y = sinkFractionVelocity_y * fractionMass;

        sink->m_state.momentum[0] += sinkFractionMomentum_x;
        sink->m_state.momentum[1] += sinkFractionMomentum_y;
    }

    if (sourceCrossSection != 0 && sourceMass != 0) {
        const double sourceFractionVelocity =
            clamp((fractionVolume / sourceCrossSection) / params.dt, 0.0, c_source);
        const double sourceFractionVelocity_squared = sourceFractionVelocity * sourceFractionVelocity;
        const double sourceFractionVelocity_x = sourceFractionVelocity * dx;
        const double sourceFractionVelocity_y = sourceFractionVelocity * dy;
        const double sourceFractionMomentum_x = sourceFractionVelocity_x * fractionMass;
        const double sourceFractionMomentum_y = sourceFractionVelocity_y * fractionMass;

        source->m_state.momentum[0] += sourceFractionMomentum_x;
        source->m_state.momentum[1] += sourceFractionMomentum_y;
    }

    if (sourceMass != 0) {
        // Energy conservation
        const double sourceVelocity0_x = sourceInitialMomentum_x * invSourceMass;
        const double sourceVelocity0_y = sourceInitialMomentum_y * invSourceMass;

        const double sourceVelocity1_x = source->m_state.momentum[0] * invSourceMass;
        const double sourceVelocity1_y = source->m_state.momentum[1] * invSourceMass;

        source->m_state.E_k -=
            0.5 * sourceMass
            * (sourceVelocity1_x * sourceVelocity1_x - sourceVelocity0_x * sourceVelocity0_x);

        source->m_state.E_k -=
            0.5 * sourceMass
            * (sourceVelocity1_y * sourceVelocity1_y - sourceVelocity0_y * sourceVelocity0_y);
    }

    if (sinkMass > 0) {
        const double sinkVelocity0_x = sinkInitialMomentum_x * invSinkMass;
        const double sinkVelocity0_y = sinkInitialMomentum_y * invSinkMass;

        const double sinkVelocity1_x = sink->m_state.momentum[0] * invSinkMass;
        const double sinkVelocity1_y = sink->m_state.momentum[1] * invSinkMass;

        sink->m_state.E_k -=
            0.5 * sinkMass
            * (sinkVelocity1_x * sinkVelocity1_x - sinkVelocity0_x * sinkVelocity0_x);

        sink->m_state.E_k -=
            0.5 * sinkMass
            * (sinkVelocity1_y * sinkVelocity1_y - sinkVelocity0_y * sinkVelocity0_y);
    }

    if (sink->m_state.E_k < 0) {
        sink->m_state.E_k = 0;
    }

    if (source->m_state.E_k < 0) {
        source->m_state.E_k = 0;
    }

    return flow * direction;
}

double GasSystem::flow(double k_flow, double dt, double P_env, double T_env, const Mix &mix) {
    if (m_variableProperties) {
        if (dt <= 0) return 0;
        const bool outgoing = pressure() > P_env;
        const double gamma = outgoing ? heatCapacityRatio() : 1 + constants::R / mixtureCv(T_env, mix);
        double amount = dt * flowRate(k_flow, pressure(), P_env, temperature(), T_env, gamma,
            std::pow(2/(gamma+1), gamma/(gamma-1)),
            std::sqrt(gamma)*std::pow(2/(gamma+1), (gamma+1)/(2*(gamma-1))));
        amount *= std::sqrt(units::AirMolecularMass / molecularMass(outgoing ? m_state.mix : mix));
        if (outgoing) amount = (std::min)(amount, 0.9*n());
        const double donorEnergy = outgoing ? (n() > 0 ? kineticEnergyPerMol() : 0) : mixtureEnergy(T_env, mix);
        const auto transfer = [&](GasSystem &g, double a) {
            if (a < 0) {
                const double before = g.bulkKineticEnergy();
                g.gainN(-a, donorEnergy, mix);
                // Incoming stationary gas dilutes momentum; lost bulk KE heats gas.
                g.changeEnergy(before - g.bulkKineticEnergy());
            } else if (a > 0 && g.n() > 0) {
                const double fraction = a/g.n();
                g.loseN(a, donorEnergy);
                g.m_state.momentum[0] *= 1-fraction;
                g.m_state.momentum[1] *= 1-fraction;
            }
        };
        GasSystem trial = *this;
        transfer(trial, amount);
        if ((trial.pressure() - P_env) * (pressure() - P_env) < 0) {
            double low = 0, high = 1;
            for (int i = 0; i < 24; ++i) {
                const double factor = (low+high)/2;
                trial = *this;
                transfer(trial, amount*factor);
                if ((trial.pressure()-P_env)*(pressure()-P_env) < 0) high = factor;
                else low = factor;
            }
            amount *= low;
        }
        transfer(*this, amount);
        return amount;
    }
    const double maxFlow = pressureEquilibriumMaxFlow(P_env, T_env);
    double flow = dt * flowRate(
        k_flow,
        pressure(),
        P_env,
        temperature(),
        T_env,
        heatCapacityRatio(),
        m_chokedFlowLimit,
        m_chokedFlowFactorCached);

    if (std::abs(flow) > std::abs(maxFlow)) {
        flow = maxFlow;
    }

    if (flow < 0) {
        const double bulk_E_k_0 = bulkKineticEnergy();
        gainN(-flow, kineticEnergyPerMol(T_env, m_degreesOfFreedom), mix);
        const double bulk_E_k_1 = bulkKineticEnergy();

        m_state.E_k += (bulk_E_k_1 - bulk_E_k_0);
    }
    else {
        const double starting_n = n();
        loseN(flow, kineticEnergyPerMol());

        m_state.momentum[0] -= (flow / starting_n) * m_state.momentum[0];
        m_state.momentum[1] -= (flow / starting_n) * m_state.momentum[1];
    }

    return flow;
}

double GasSystem::pressureEquilibriumMaxFlow(const GasSystem *b) const {
    // pressure_a = (kineticEnergy() + n * b->kineticEnergyPerMol()) / (0.5 * degreesOfFreedom * volume())
    // pressure_b = (b->kineticEnergy() - n *  / (0.5 * b->degreesOfFreedom * b->volume())
    // pressure_a = pressure_b

    // E_a = kineticEnergy()
    // E_b = b->kineticEnergy()
    // D_a = E_a / n()
    // D_b = E_b / b->n()
    // Q_a = 1 / (0.5 * degreesOfFreedom * volume())
    // Q_b = 1 / (0.5 * b->degreesOfFreedom * b->volume())
    // pressure_a = Q_a * (E_a + dn * D_b)
    // pressure_b = Q_b * (E_b - dn * D_b)

    if (pressure() > b->pressure()) {
        const double maxFlow =
                (b->volume() * kineticEnergy() - volume() * b->kineticEnergy()) /
                (b->volume() * kineticEnergyPerMol() + volume() * kineticEnergyPerMol());
        return std::fmax(0.0, std::fmin(maxFlow, n()));
    }
    else {
        const double maxFlow =
                (b->volume() * kineticEnergy() - volume() * b->kineticEnergy()) /
                (b->volume() * b->kineticEnergyPerMol() + volume() * b->kineticEnergyPerMol());
        return std::fmin(0.0, std::fmax(maxFlow, -b->n()));
    }
}

double GasSystem::pressureEquilibriumMaxFlow(double P_env, double T_env) const {
    if (pressure() > P_env) {
        return -(P_env * (0.5 * m_degreesOfFreedom * volume()) - kineticEnergy()) / kineticEnergyPerMol();
    }
    else {
        const double E_k_per_mol_env = 0.5 * T_env * constants::R * m_degreesOfFreedom;
        return -(P_env * (0.5 * m_degreesOfFreedom * volume()) - kineticEnergy()) / E_k_per_mol_env;
    }
}
