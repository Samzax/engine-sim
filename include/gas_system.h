#ifndef ATG_ENGINE_SIM_GAS_SYSTEM_H
#define ATG_ENGINE_SIM_GAS_SYSTEM_H

#include "constants.h"
#include "units.h"
#include "gas_execution.h"

#include <cfloat>
#include <cmath>

class GasSystem {
    friend class GasPipe;
    public:
        struct Mix {
            double p_fuel = 0.0;
            double p_inert = 1.0;
            double p_o2 = 0.0;
            // Products are a subset of p_inert, retained for old script/gauge APIs.
            double p_co2 = 0.0;
            double p_h2o = 0.0;
            double residualFraction = 0.0; // passive burned-gas mass fraction
            double fuelMolecularMass = 0.114232; // kg/mol
            double oxygenPerFuel = 12.5;
        };

        struct State {
            double n_mol = 0.0;
            double E_k = 0.0;
            double V = 0.0;
            double momentum[2] = { 0.0, 0.0 };

            Mix mix;
        };

        struct FlowParameters {
            double k_flow;
            double dt;
            double direction_x, direction_y;
            double crossSectionArea_0, crossSectionArea_1;
            GasSystem *system_0, *system_1;
        };

    public:
        GasSystem() = default;
        ~GasSystem() = default;

        ES_GAS_FUNCTION void setGeometry(double width, double height, double dx, double dy);
        ES_GAS_FUNCTION void initialize(double P, double V, double T, const Mix &mix = {}, int degreesOfFreedom = 5);
        ES_GAS_FUNCTION void reset(double P, double T, const Mix &mix = {});
        ES_GAS_FUNCTION void setVariableProperties(bool enabled);
        ES_GAS_FUNCTION bool variableProperties() const { return m_variableProperties; }
        ES_GAS_FUNCTION static double molecularMass(const Mix &mix);
        ES_GAS_FUNCTION static double mixtureEnergy(double temperature, const Mix &mix);
        ES_GAS_FUNCTION static double mixtureCv(double temperature, const Mix &mix);
        ES_GAS_FUNCTION double energyAtTemperature(double temperature) const;
        ES_GAS_FUNCTION double heatCapacity(double temperature) const;
        ES_GAS_FUNCTION double molarEnergy(double temperature) const;
        ES_GAS_FUNCTION double molarCv(double temperature) const;

        ES_GAS_FUNCTION void setVolume(double V);
        ES_GAS_FUNCTION void setN(double n);

        ES_GAS_FUNCTION void changeVolume(double dV);
        ES_GAS_FUNCTION void changePressure(double pressure);
        ES_GAS_FUNCTION void changeTemperature(double dT);
        ES_GAS_FUNCTION void changeTemperature(double dT, double n);
        ES_GAS_FUNCTION void changeEnergy(double dE);
        ES_GAS_FUNCTION void changeMix(const Mix &mix);
        ES_GAS_FUNCTION void injectFuel(double n);

        ES_GAS_FUNCTION double react(double n, const Mix &mix);
        ES_GAS_FUNCTION static double flowConstant(double flowRate, double P, double pressureDrop, double T, double hcr);
        ES_GAS_FUNCTION static double k_28inH2O(double flowRateScfm);
        ES_GAS_FUNCTION static double k_carb(double flowRateScfm);
        ES_GAS_FUNCTION static double flowRate(
            double k_flow,
            double P0,
            double P1,
            double T0,
            double T1,
            double hcr,
            double chokedFlowLimit,
            double chokedFlowRateCached);
        ES_GAS_FUNCTION double loseN(double dn, double E_k_per_mol);
        ES_GAS_FUNCTION double gainN(double dn, double E_k_per_mol, const Mix &mix = {});
        ES_GAS_FUNCTION void dissipateExcessVelocity();

        ES_GAS_FUNCTION void updateVelocity(double dt, double beta = 1.0);
        ES_GAS_FUNCTION void dissipateVelocity(double dt, double timeConstant);

        ES_GAS_FUNCTION static double flow(const FlowParameters &params);
        ES_GAS_FUNCTION double flow(double k_flow, double dt, double P_env, double T_env, const Mix &mix = {});

        ES_GAS_FUNCTION double pressureEquilibriumMaxFlow(const GasSystem *b) const;
        ES_GAS_FUNCTION double pressureEquilibriumMaxFlow(double P_env, double T_env) const;

        ES_GAS_FUNCTION inline static constexpr double kineticEnergyPerMol(double T, int degreesOfFreedom);
        ES_GAS_FUNCTION inline static constexpr double heatCapacityRatio(int degreesOfFreedom);
        ES_GAS_FUNCTION inline static double chokedFlowLimit(int degreesOfFreedom);
        ES_GAS_FUNCTION inline static double chokedFlowRate(int degreesOfFreedom);

        ES_GAS_FUNCTION inline double approximateDensity() const;
        ES_GAS_FUNCTION inline int degreesOfFreedom() const { return m_degreesOfFreedom; }
        ES_GAS_FUNCTION inline double n() const;
        ES_GAS_FUNCTION inline double n(double V) const;
        ES_GAS_FUNCTION inline double kineticEnergy() const;
        ES_GAS_FUNCTION inline double kineticEnergy(double n) const;
        ES_GAS_FUNCTION inline double kineticEnergyPerMol() const { return kineticEnergy(1.0); }
        ES_GAS_FUNCTION inline double totalEnergy() const;
        ES_GAS_FUNCTION inline double bulkKineticEnergy() const;
        ES_GAS_FUNCTION inline double c() const;
        ES_GAS_FUNCTION inline double dynamicPressure(double dx, double dy) const;
        ES_GAS_FUNCTION inline double mass() const;
        ES_GAS_FUNCTION inline double pressure() const;
        ES_GAS_FUNCTION double temperature() const;
        ES_GAS_FUNCTION inline double velocity_x() const;
        ES_GAS_FUNCTION inline double velocity_y() const;
        ES_GAS_FUNCTION inline double volume() const;
        ES_GAS_FUNCTION inline double volume(double n) const;
        ES_GAS_FUNCTION inline double n_fuel() const;
        ES_GAS_FUNCTION inline double n_inert() const;
        ES_GAS_FUNCTION inline double n_o2() const;
        ES_GAS_FUNCTION inline double heatCapacityRatio() const;
        ES_GAS_FUNCTION inline Mix mix() const { return m_state.mix; }

    protected:
        State m_state;

        int m_degreesOfFreedom = 5;

        double m_chokedFlowLimit = 0;
        double m_chokedFlowFactorCached = 0;

        double m_width = 0.0;
        double m_height = 0.0;
        double m_dx = 0.0;
        double m_dy = 0.0;
        bool m_variableProperties = false;
        mutable double m_cachedEnergy = -1, m_cachedN = -1, m_cachedTemperature = 300;
        ES_GAS_FUNCTION void refreshProperties() const;
        mutable bool m_propertiesValid = false, m_massPropertiesValid = false;
        ES_GAS_FUNCTION void invalidateProperties() const { m_propertiesValid=false; m_massPropertiesValid=false; }
        ES_GAS_FUNCTION void refreshMass() const {
            if (!m_massPropertiesValid) {
                m_molarMass=molecularMass(m_state.mix);
                m_massPropertiesValid=true;
            }
        }
        mutable double m_lowCp[5]{}, m_highCp[5]{};
        mutable double m_u200 = 0, m_u1000 = 0, m_u6000 = 0;
        mutable double m_cv200 = 0, m_cv6000 = 0, m_molarMass = 0;
};

ES_GAS_FUNCTION inline constexpr double GasSystem::kineticEnergyPerMol(double T, int degreesOfFreedom) {
    return 0.5 * T * constants::R * degreesOfFreedom;
}

ES_GAS_FUNCTION inline constexpr double GasSystem::heatCapacityRatio(int degreesOfFreedom) {
    return 1.0 + (2.0 / degreesOfFreedom);
}

ES_GAS_FUNCTION inline double GasSystem::chokedFlowLimit(int degreesOfFreedom) {
    const double hcr = heatCapacityRatio(degreesOfFreedom);
    return std::pow((2.0 / (hcr + 1)), hcr / (hcr - 1));
}

ES_GAS_FUNCTION inline double GasSystem::chokedFlowRate(int degreesOfFreedom) {
    const double hcr = heatCapacityRatio(degreesOfFreedom);
    double flowRate =
        std::sqrt(hcr) * std::pow(2 / (hcr + 1), (hcr + 1) / (2 * (hcr - 1)));

    return flowRate;
}

ES_GAS_FUNCTION inline double GasSystem::approximateDensity() const {
    return mass() / volume();
}

ES_GAS_FUNCTION inline double GasSystem::n() const {
    return m_state.n_mol;
}

ES_GAS_FUNCTION inline double GasSystem::n(double V) const {
    return (V / volume()) * n();
}

ES_GAS_FUNCTION inline double GasSystem::kineticEnergy() const {
    return m_state.E_k;
}

ES_GAS_FUNCTION inline double GasSystem::kineticEnergy(double n) const {
    return (kineticEnergy() / this->n()) * n;
}

ES_GAS_FUNCTION inline double GasSystem::c() const {
    if (n() == 0 || kineticEnergy() == 0) return 0;

    const double hcr = heatCapacityRatio();
    const double staticPressure = pressure();
    const double density = approximateDensity();
    const double c = std::sqrt(staticPressure * hcr / density);

    return c;
}

ES_GAS_FUNCTION inline double GasSystem::totalEnergy() const {
    if (n() == 0) return 0;

    const double invMass = 1 / mass();
    const double v_x = m_state.momentum[0] * invMass;
    const double v_y = m_state.momentum[1] * invMass;
    const double v_squared = v_x * v_x + v_y * v_y;

    return kineticEnergy() + 0.5 * mass() * v_squared;
}

ES_GAS_FUNCTION inline double GasSystem::bulkKineticEnergy() const {
    const double m = mass();
    if (m == 0) return 0;

    const double v_x = m_state.momentum[0] / m;
    const double v_y = m_state.momentum[1] / m;
    const double v_squared = v_x * v_x + v_y * v_y;
    return 0.5 * m * v_squared;
}

ES_GAS_FUNCTION inline double GasSystem::dynamicPressure(double dx, double dy) const {
    if (n() == 0 || kineticEnergy() == 0) return 0;

    const double inverseMass = 1 / this->mass();
    const double v = inverseMass * (dx * m_state.momentum[0] + dy * m_state.momentum[1]);

    if (v <= 0) {
        return 0;
    }

    const double hcr = heatCapacityRatio();
    const double staticPressure = pressure();
    const double density = approximateDensity();
    const double c_squared = staticPressure * hcr / density;
    const double machNumber_squared = v * v / c_squared;

    // Below is equivalent to:
    // staticPressure * pow(1 + ((hcr - 1) / 2) * machNumber * machNumber, hcr / (hcr - 1)) - 1)

    const double x = 1 + ((hcr - 1) / 2) * machNumber_squared;
    if (m_variableProperties) return staticPressure * (std::pow(x, hcr / (hcr - 1)) - 1);
    double x_d;
    switch (m_degreesOfFreedom) {
    case 3:
        x_d = x * x * x * x * x;
        break;
    case 5:
    {
        const double x_2 = x * x;
        const double x_3 = x_2 * x;
        x_d = x_3 * x_3 * x;
        break;
    }
    default:
        x_d = x;
    }

    return staticPressure * (std::sqrt(x_d) - 1);
}

ES_GAS_FUNCTION inline double GasSystem::mass() const {
    if (m_variableProperties) { refreshMass(); return m_molarMass * n(); }
    return units::AirMolecularMass * n();
}

ES_GAS_FUNCTION inline double GasSystem::pressure() const {
    if (m_variableProperties) return volume() > 0 ? n() * constants::R * temperature() / volume() : 0;
    const double volume = this->volume();
    return (volume != 0)
        ? kineticEnergy() / (0.5 * m_degreesOfFreedom * volume)
        : 0;
}


ES_GAS_FUNCTION inline double GasSystem::velocity_x() const {
    if (n() == 0) return 0;
    else return m_state.momentum[0] / mass();
}

ES_GAS_FUNCTION inline double GasSystem::velocity_y() const {
    if (n() == 0) return 0;
    else return m_state.momentum[1] / mass();
}

ES_GAS_FUNCTION inline double GasSystem::volume() const {
    return m_state.V;
}

ES_GAS_FUNCTION inline double GasSystem::volume(double n) const {
    return n * this->n() / volume();
}

ES_GAS_FUNCTION inline double GasSystem::n_fuel() const {
    return m_state.mix.p_fuel * n();
}

ES_GAS_FUNCTION inline double GasSystem::n_inert() const {
    return m_state.mix.p_inert * n();
}

ES_GAS_FUNCTION inline double GasSystem::n_o2() const {
    return m_state.mix.p_o2 * n();
}

ES_GAS_FUNCTION inline double GasSystem::heatCapacityRatio() const {
    if (m_variableProperties) return 1 + constants::R / molarCv(temperature());
    return heatCapacityRatio(m_degreesOfFreedom);
}

#endif /* ATG_ENGINE_SIM_GAS_SYSTEM_H */
