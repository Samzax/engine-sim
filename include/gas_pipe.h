#ifndef ATG_ENGINE_SIM_GAS_PIPE_H
#define ATG_ENGINE_SIM_GAS_PIPE_H

#include "gas_system.h"
#include <array>
#include <vector>
#include <algorithm>
#include <stdexcept>

// First-order finite-volume 1D Euler pipe, with Rusanov interface fluxes.
// Ports exchange gas with existing calibrated orifices; the distributed interior
// advances conservatively. Reflecting end pressure fluxes close the split step.
class GasPipe {
public:
    void initialize(const GasSystem &prototype, double length, double area, int count, double frictionFactor=0.02) {
        if (count<1 || count>64) throw std::invalid_argument("pipe_cells must be between 1 and 64");
        if (!std::isfinite(frictionFactor) || frictionFactor<0)
            throw std::invalid_argument("pipe_friction_factor must be finite and nonnegative");
        m_frictionFactor=frictionFactor;
        m_cells.clear();
        if (count==1) return;
        m_area=area; m_dx=length/count;
        m_cells.assign(count,prototype);
        m_fluxes.resize(count+1);
        m_states.resize(count);
        for (auto &cell:m_cells) {
            cell.m_state.V/=count; cell.m_state.n_mol/=count;
            cell.m_state.E_k/=count;
            cell.m_state.momentum[0]/=count; cell.m_state.momentum[1]=0;
            cell.setGeometry(m_dx,std::sqrt(area),1,0);
            cell.m_cachedEnergy=-1;
        }
    }
    bool active() const {return !m_cells.empty();}
    GasSystem &first(){return m_cells.front();}
    GasSystem &last(){return m_cells.back();}
    GasSystem &cell(int i){return m_cells.at(i);}
    int count() const {return static_cast<int>(m_cells.size());}
    void configure(bool enabled, double fuelMass, double oxygenPerFuel) {
        for (auto &cell:m_cells) {
            auto mix=cell.mix();
            if (enabled) { mix.p_inert=0.79; mix.p_o2=0.21; }
            mix.fuelMolecularMass=fuelMass; mix.oxygenPerFuel=oxygenPerFuel;
            cell.changeMix(mix);
            cell.setVariableProperties(enabled);
        }
    }
    double stableTimestep() const {
        double speed=1;
        for (const auto &cell:m_cells) speed=(std::max)(speed,std::abs(cell.velocity_x())+cell.c());
        return 0.25*m_dx/speed;
    }

    void advance(double dt) {
        if (!active()) return;
        int steps=0;
        while (dt>0) {
            if (++steps>10000) throw std::runtime_error("Pipe timestep requires excessive substeps; increase simulation frequency");
            const double h=(std::min)(dt,stableTimestep());
            const int n=count();
            for (int i=0;i<n;++i) m_states[i]=conserved(m_cells[i]);
            m_fluxes[0]={}; m_fluxes[n]={};
            m_fluxes[0][5]=m_cells.front().pressure();
            m_fluxes[n][5]=m_cells.back().pressure();
            for (int i=1;i<n;++i) {
                const auto &left=m_cells[i-1]; const auto &right=m_cells[i];
                const auto fl=flux(left,m_states[i-1]), fr=flux(right,m_states[i]);
                const double a=(std::max)(std::abs(left.velocity_x())+left.c(), std::abs(right.velocity_x())+right.c());
                for (int k=0;k<8;++k)
                    m_fluxes[i][k]=0.5*(fl[k]+fr[k])-0.5*a*(m_states[i][k]-m_states[i-1][k]);
            }
            for (int i=0;i<n;++i) {
                for (int k=0;k<8;++k) m_states[i][k]-=h/m_dx*(m_fluxes[i+1][k]-m_fluxes[i][k]);
                restore(m_cells[i],m_states[i]);
                // Darcy wall friction; lost bulk energy remains as gas heat.
                const double oldBulk=m_cells[i].bulkKineticEnergy();
                const double diameter=2*std::sqrt(m_area/constants::pi);
                m_cells[i].m_state.momentum[0]/=1+m_frictionFactor*std::abs(m_cells[i].velocity_x())*h/(2*diameter);
                m_cells[i].changeEnergy(oldBulk-m_cells[i].bulkKineticEnergy());
            }
            dt-=h;
        }
    }

    void aggregate(GasSystem &out) const {
        if (!active()) return;
        out=m_cells.front();
        double energy=0, momentum=0, residualMass=0;
        std::array<double,5> species{};
        for (const auto &cell:m_cells) {
            const auto u=conserved(cell);
            for (int k=0;k<5;++k) species[k]+=u[k]*cell.volume();
            energy+=cell.totalEnergy(); momentum+=cell.m_state.momentum[0];
            residualMass+=cell.mass()*cell.mix().residualFraction;
        }
        out.m_state.V=m_area*m_dx*count();
        Vector u{};
        for (int k=0;k<5;++k) u[k]=species[k]/out.volume();
        u[5]=momentum/out.volume(); u[6]=energy/out.volume(); u[7]=residualMass/out.volume();
        restore(out,u);
    }

private:
    // Extensive species moles, axial momentum, total energy and burned-gas mass
    // per m^3. One fuel definition per engine is shared by every cell.
    using Vector=std::array<double,8>;
    static Vector conserved(const GasSystem &g) {
        const auto m=g.mix(); const double c=g.n()/g.volume();
        return {c*m.p_fuel,c*m.p_o2,c*(m.p_inert-m.p_co2-m.p_h2o),c*m.p_co2,c*m.p_h2o,
            g.m_state.momentum[0]/g.volume(),g.totalEnergy()/g.volume(),g.mass()*m.residualFraction/g.volume()};
    }
    static Vector flux(const GasSystem &g,const Vector &u) {
        Vector f{}; const double v=g.velocity_x(), p=g.pressure();
        for (int k=0;k<8;++k) f[k]=u[k]*v;
        f[5]+=p; f[6]+=p*v;
        return f;
    }
    static void restore(GasSystem &g,const Vector &u) {
        double molarDensity=0;
        for (int k=0;k<5;++k) {
            if (!std::isfinite(u[k]) || u[k]<-1e-10)
                throw std::runtime_error("Pipe species positivity failure");
            molarDensity+=(std::max)(0.0,u[k]);
        }
        if (!(molarDensity>0)) throw std::runtime_error("Pipe lost gas density");
        auto &m=g.m_state.mix;
        m.p_fuel=(std::max)(0.0,u[0])/molarDensity;
        m.p_o2=(std::max)(0.0,u[1])/molarDensity;
        m.p_co2=(std::max)(0.0,u[3])/molarDensity;
        m.p_h2o=(std::max)(0.0,u[4])/molarDensity;
        m.p_inert=((std::max)(0.0,u[2])+(std::max)(0.0,u[3])+(std::max)(0.0,u[4]))/molarDensity;
        g.m_state.n_mol=molarDensity*g.volume();
        g.m_state.momentum[0]=u[5]*g.volume(); g.m_state.momentum[1]=0;
        g.m_propertiesValid=false; g.m_cachedEnergy=-1;
        m.residualFraction=std::clamp(u[7]*g.volume()/g.mass(),0.0,1.0);
        g.m_state.E_k=u[6]*g.volume()-g.bulkKineticEnergy();
        if (!std::isfinite(g.m_state.E_k) || g.m_state.E_k<=0)
            throw std::runtime_error("Pipe internal energy positivity failure");
    }
    std::vector<GasSystem> m_cells;
    std::vector<Vector> m_fluxes,m_states;
    double m_area=1,m_dx=1,m_frictionFactor=0.02;
};
#endif
