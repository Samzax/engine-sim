#ifndef ATG_ENGINE_SIM_CYLINDER_THERMAL_MODEL_H
#define ATG_ENGINE_SIM_CYLINDER_THERMAL_MODEL_H

#include "gas_system.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Per-cylinder lumped metal temperature coupled to a prescribed coolant reservoir.
class CylinderThermalModel {
public:
    struct Parameters {
        bool enabled = true;
        double initialWallTemperature = 363.15; // K; warm start by default
        double coolantTemperature = 363.15; // K
        double wallHeatCapacity = 4000.0; // J/K per cylinder
        double coolantConductance = 40.0; // W/K per cylinder
        double heatTransferScale = 1.0; // calibration multiplier
    };

    void initialize(const Parameters &p) {
        const auto positive = [](double x) { return std::isfinite(x) && x > 0; };
        const auto nonnegative = [](double x) { return std::isfinite(x) && x >= 0; };
        if (!positive(p.initialWallTemperature) || !positive(p.coolantTemperature)
            || !positive(p.wallHeatCapacity) || !nonnegative(p.coolantConductance)
            || !nonnegative(p.heatTransferScale))
            throw std::invalid_argument("Thermal model requires positive finite temperatures (K) and wall heat capacity, and nonnegative finite conductance and heat-transfer scale");
        m_parameters = p;
        m_wallTemperature = p.enabled ? p.initialWallTemperature : 363.15;
        m_coolantEnergy = 0;
    }

    // Hohenberg: V in m^3, p in bar, T in K, mean piston speed in m/s.
    ES_GAS_FUNCTION static double coefficient(double volume, double pressure, double temperature,
        double meanPistonSpeed) {
        if (volume <= 0 || pressure <= 0 || temperature <= 0) return 0;
        return 130.0 * std::pow(volume, -0.06) * std::pow(pressure / 1e5, 0.8)
            * std::pow(temperature, -0.4) * std::pow(std::abs(meanPistonSpeed) + 1.4, 0.8);
    }

    ES_GAS_FUNCTION void exchange(GasSystem &gas, double area, double meanPistonSpeed, double dt) {
        if (dt <= 0) return;
        if (!m_parameters.enabled) {
            gas.changeEnergy((363.15 - gas.temperature()) * area * 100.0 * dt);
            return;
        }
        const double tg = gas.temperature();
        const double cg = tg > 0 ? gas.kineticEnergy() / tg : 0;
        const double conductance = m_parameters.heatTransferScale * area
            * coefficient(gas.volume(), gas.pressure(), tg, meanPistonSpeed);
        const double gh = dt * conductance;
        const double gc = dt * m_parameters.coolantConductance;
        const double cw = m_parameters.wallHeatCapacity;
        if (gas.variableProperties() && cg > 0) {
            const double initialEnergy = gas.kineticEnergy();
            const double oldWall = m_wallTemperature;
            double low = (std::min)((std::min)(tg, oldWall), m_parameters.coolantTemperature);
            double high = (std::max)((std::max)(tg, oldWall), m_parameters.coolantTemperature);
            double t = tg;
            for (int i = 0; i < 32; ++i) {
                const double delta = gas.energyAtTemperature(t) - initialEnergy;
                const double wall = (cw*oldWall + gc*m_parameters.coolantTemperature - delta)/(cw+gc);
                const double residual = delta - gh*(wall-t);
                if (std::abs(residual) < 1e-10 * (std::max)(1.0, initialEnergy)) break;
                if (residual > 0) high = t; else low = t;
                const double derivative = gas.heatCapacity(t)*(1+gh/(cw+gc)) + gh;
                const double next = t-residual/derivative;
                t = next > low && next < high ? next : (low+high)/2;
            }
            const double delta = gas.energyAtTemperature(t)-initialEnergy;
            m_wallTemperature = (cw*oldWall + gc*m_parameters.coolantTemperature-delta)/(cw+gc);
            gas.changeEnergy(delta);
            m_coolantEnergy += gc*(m_wallTemperature-m_parameters.coolantTemperature);
            return;
        }
        // Backward Euler for the coupled gas/metal system. Eliminating the gas
        // unknown makes the wall update a positive weighted temperature average.
        const double coupling = cg > 0 ? gh / (1.0 + gh / cg) : 0;
        const double tw = (cw * m_wallTemperature + coupling * tg
            + gc * m_parameters.coolantTemperature) / (cw + coupling + gc);
        gas.changeEnergy(coupling * (tw - tg));
        m_coolantEnergy += gc * (tw - m_parameters.coolantTemperature);
        m_wallTemperature = tw;
    }

    ES_GAS_FUNCTION double wallTemperature() const { return m_wallTemperature; }
    ES_GAS_FUNCTION double coolantEnergy() const { return m_coolantEnergy; }
    ES_GAS_FUNCTION double wallHeatCapacity() const { return m_parameters.wallHeatCapacity; }
    ES_GAS_FUNCTION double coolantTemperature() const { return m_parameters.coolantTemperature; }
    ES_GAS_FUNCTION void addWallEnergy(double energy) {
        if (m_parameters.enabled) m_wallTemperature += energy/m_parameters.wallHeatCapacity;
    }

private:
    Parameters m_parameters;
    double m_wallTemperature = 363.15;
    double m_coolantEnergy = 0;
};

#endif
