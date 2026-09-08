#ifndef ATG_ENGINE_SIM_LUBRICATION_MODEL_H
#define ATG_ENGINE_SIM_LUBRICATION_MODEL_H

#include "cylinder_thermal_model.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

class LubricationModel {
public:
    struct Parameters {
        bool enabled = true;
        double initialTemperature = 363.15; // K
        double viscosity40 = 80.0, viscosity100 = 10.5; // cSt, provisional oil
        double heatCapacity = 2000.0; // J/K per cylinder's share of oil
        double wallConductance = 10.0, coolantConductance = 5.0; // W/K
        double frictionScale = 1.0;
    };

    void initialize(const Parameters &p) {
        const auto positive=[](double x){return std::isfinite(x)&&x>0;};
        const auto nonnegative=[](double x){return std::isfinite(x)&&x>=0;};
        if (!positive(p.initialTemperature) || !positive(p.heatCapacity)
            || !std::isfinite(p.viscosity40) || !std::isfinite(p.viscosity100)
            || p.viscosity100<=0.3 || p.viscosity40<=p.viscosity100
            || !nonnegative(p.wallConductance) || !nonnegative(p.coolantConductance)
            || !nonnegative(p.frictionScale))
            throw std::invalid_argument("Oil model requires positive finite temperature/capacity, viscosity40 > viscosity100 > 0.3 cSt, and nonnegative finite conductances/friction scale");
        m_parameters=p;
        m_temperature=p.initialTemperature;
        m_coolantEnergy=m_frictionEnergy=0;
        // Walther two-point viscosity relation, temperatures in Kelvin.
        const double y40=std::log10(std::log10(p.viscosity40+0.7));
        const double y100=std::log10(std::log10(p.viscosity100+0.7));
        m_b=(y40-y100)/(std::log10(373.15)-std::log10(313.15));
        m_a=y40+m_b*std::log10(313.15);
        m_referenceViscosity=viscosity(363.15);
        updateRatio();
    }

    double viscosity(double t) const {
        t=std::clamp(t,250.0,450.0);
        return std::pow(10.0,std::pow(10.0,m_a-m_b*std::log10(t)))-0.7;
    }
    double viscousMultiplier() const { return m_parameters.enabled ? m_ratio : 1; }
    double frictionScale() const { return m_parameters.frictionScale; }
    double temperature() const { return m_temperature; }
    double coolantEnergy() const { return m_coolantEnergy; }
    double frictionEnergy() const { return m_frictionEnergy; }

    void advance(double dt, double frictionPower, CylinderThermalModel &wall) {
        if (!m_parameters.enabled || dt<=0) return;
        const double gh=dt*m_parameters.wallConductance;
        const double coupling=gh/(1+gh/wall.wallHeatCapacity());
        const double gc=dt*m_parameters.coolantConductance;
        const double work=(std::max)(0.0,frictionPower)*dt;
        const double t=(m_parameters.heatCapacity*m_temperature+coupling*wall.wallTemperature()
            +gc*wall.coolantTemperature()+work)/(m_parameters.heatCapacity+coupling+gc);
        const double fromWall=coupling*(wall.wallTemperature()-t);
        wall.addWallEnergy(-fromWall);
        m_coolantEnergy+=gc*(t-wall.coolantTemperature());
        m_frictionEnergy+=work;
        m_temperature=t;
        updateRatio();
    }

private:
    void updateRatio(){m_ratio=viscosity(m_temperature)/m_referenceViscosity;}
    Parameters m_parameters;
    double m_temperature=363.15, m_coolantEnergy=0, m_frictionEnergy=0;
    double m_a=0, m_b=0, m_referenceViscosity=1, m_ratio=1;
};
#endif
