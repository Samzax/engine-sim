#ifndef ATG_ENGINE_SIM_CYLINDER_THERMAL_MODEL_H
#define ATG_ENGINE_SIM_CYLINDER_THERMAL_MODEL_H

#include "gas_system.h"
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
    static double coefficient(double volume, double pressure, double temperature,
        double meanPistonSpeed) {
        if (volume <= 0 || pressure <= 0 || temperature <= 0) return 0;
        return 130.0 * std::pow(volume, -0.06) * std::pow(pressure / 1e5, 0.8)
            * std::pow(temperature, -0.4) * std::pow(std::abs(meanPistonSpeed) + 1.4, 0.8);
    }

    void exchange(GasSystem &gas, double area, double meanPistonSpeed, double dt) {
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
        // Backward Euler for the coupled gas/metal system. Eliminating the gas
        // unknown makes the wall update a positive weighted temperature average.
        const double coupling = cg > 0 ? gh / (1.0 + gh / cg) : 0;
        const double tw = (cw * m_wallTemperature + coupling * tg
            + gc * m_parameters.coolantTemperature) / (cw + coupling + gc);
        gas.changeEnergy(coupling * (tw - tg));
        m_coolantEnergy += gc * (tw - m_parameters.coolantTemperature);
        m_wallTemperature = tw;
    }

    double wallTemperature() const { return m_wallTemperature; }
    double coolantEnergy() const { return m_coolantEnergy; }

private:
    Parameters m_parameters;
    double m_wallTemperature = 363.15;
    double m_coolantEnergy = 0;
};

#endif
