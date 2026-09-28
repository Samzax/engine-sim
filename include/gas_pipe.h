#ifndef ATG_ENGINE_SIM_GAS_PIPE_H
#define ATG_ENGINE_SIM_GAS_PIPE_H

#include "gas_system.h"
#include "gas_transport.h"
#include "gpu_pipe.h"
#include "simulation_profile.h"
#include <array>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cstring>
#if defined(__AVX2__)
#include <immintrin.h>
#endif

// First-order finite-volume 1D Euler pipe, with Rusanov interface fluxes.
// Ports exchange gas with existing calibrated orifices; the distributed interior
// advances conservatively. Reflecting end pressure fluxes close the split step.
class GasPipe {
public:
    void initialize(const GasSystem &prototype, double length, double area, int count, double frictionFactor=0.02) {
        if (count<1 || count>64) throw std::invalid_argument("pipe_cells must be between 1 and 64");
        if (!std::isfinite(frictionFactor) || frictionFactor<0)
            throw std::invalid_argument("pipe_friction_factor must be finite and nonnegative");
        m_cachedCflValid=false;
        m_frictionFactor=frictionFactor;
        m_cells.clear();
        if (count==1) return;
        m_area=area; m_dx=length/count;
        m_cells.assign(count,prototype);
        m_fluxes.resize(count+1);
        m_states.resize(count);
        m_v.resize(count); m_p.resize(count); m_s.resize(count);
        for (auto &cell:m_cells) {
            cell.m_state.V/=count; cell.m_state.n_mol/=count;
            cell.m_state.E_k/=count;
            cell.m_state.momentum[0]/=count; cell.m_state.momentum[1]=0;
            cell.setGeometry(m_dx,std::sqrt(area),1,0);
            cell.m_cachedEnergy=-1;
        }
    }
    void exportCoupled(gpu_pipe::Pipe &out,GasSystem *cells) const {
        out.count=count(); out.dx=m_dx; out.diameter=2*std::sqrt(m_area/constants::pi);
        out.friction=m_frictionFactor; out.fuelMass=m_cells.front().mix().fuelMolecularMass;
        out.stableTimestep=stableTimestep();
        std::copy(m_cells.begin(),m_cells.end(),cells);
    }
    void importCoupled(const GasSystem *cells,double cfl) {
        std::copy(cells,cells+count(),m_cells.begin());
        m_cachedCflStates.resize(count());
        for(int i=0;i<count();++i) std::memcpy(&m_cachedCflStates[i],&m_cells[i].m_state,sizeof(GasSystem::State));
        m_cachedCflStep=cfl; m_cachedCflValid=true;
    }
    bool active() const {return !m_cells.empty();}
    GasSystem &first(){return m_cells.front();}
    GasSystem &last(){return m_cells.back();}
    GasSystem &cell(int i){return m_cells.at(i);}
    int count() const {return static_cast<int>(m_cells.size());}
    void configure(bool enabled, double fuelMass, double oxygenPerFuel) {
        m_cachedCflValid=false;
        for (auto &cell:m_cells) {
            auto mix=cell.mix();
            if (enabled) { mix.p_inert=0.79; mix.p_o2=0.21; }
            mix.fuelMolecularMass=fuelMass; mix.oxygenPerFuel=oxygenPerFuel;
            cell.changeMix(mix);
            cell.setVariableProperties(enabled);
        }
    }
    double stableTimestep() const {
        if (m_cachedCflValid) {
            // Mutable cell references may survive a GPU call. Compare the full
            // physical state so any later port/user mutation invalidates reuse.
            bool same=m_cachedCflStates.size()==m_cells.size();
            for(size_t i=0;same && i<m_cells.size();++i)
                same=m_cells[i].m_variableProperties && std::memcmp(
                    &m_cachedCflStates[i],&m_cells[i].m_state,sizeof(GasSystem::State))==0;
            if(same) return m_cachedCflStep;
            m_cachedCflValid=false;
        }
        double speed=1;
        for (const auto &cell:m_cells) speed=(std::max)(speed,std::abs(cell.velocity_x())+cell.c());
        return 0.25*m_dx/speed;
    }

    void advance(double dt) {
        if (!active()) return;
        const double diameter=2*std::sqrt(m_area/constants::pi);
        int steps=0;
        while (dt>0) {
            if (++steps>10000) throw std::runtime_error("Pipe timestep requires excessive substeps; increase simulation frequency");
            const double h=(std::min)(dt,stableTimestep());
            const int n=count();
            // One pass of the state-derived scalars: the interface loop used to
            // evaluate velocity/pressure/sound-speed about three times per cell.
            for (int i=0;i<n;++i) {
                m_v[i]=m_cells[i].velocity_x();
                m_p[i]=m_cells[i].pressure();
                m_s[i]=(std::abs)(m_v[i])+m_cells[i].c();
            }
            for (int i=0;i<n;++i) m_states[i]=conserved(m_cells[i]);
            m_fluxes[0]={}; m_fluxes[n]={};
            m_fluxes[0][5]=m_p[0];
            m_fluxes[n][5]=m_p[n-1];
            for (int i=1;i<n;++i) {
                const double a=(std::max)(m_s[i-1],m_s[i]);
                m_fluxes[i]=combineFlux(m_states[i-1],m_v[i-1],m_p[i-1],
                    m_states[i],m_v[i],m_p[i],a);
            }
#if defined(__AVX2__)
            const __m256d hOverDx=_mm256_set1_pd(h/m_dx);
#endif
            for (int i=0;i<n;++i) {
#if defined(__AVX2__)
                // Elementwise in k: the same sub/mul/sub chain the scalar loop
                // runs, folded to one ymm pair (8 doubles = 2 lanes).
                const __m256d s0=_mm256_loadu_pd(&m_states[i][0]);
                const __m256d s1=_mm256_loadu_pd(&m_states[i][4]);
                const __m256d d0=_mm256_sub_pd(_mm256_loadu_pd(&m_fluxes[i+1][0]),
                    _mm256_loadu_pd(&m_fluxes[i][0]));
                const __m256d d1=_mm256_sub_pd(_mm256_loadu_pd(&m_fluxes[i+1][4]),
                    _mm256_loadu_pd(&m_fluxes[i][4]));
                _mm256_storeu_pd(&m_states[i][0],_mm256_sub_pd(s0,_mm256_mul_pd(hOverDx,d0)));
                _mm256_storeu_pd(&m_states[i][4],_mm256_sub_pd(s1,_mm256_mul_pd(hOverDx,d1)));
#else
                for (int k=0;k<8;++k) m_states[i][k]-=h/m_dx*(m_fluxes[i+1][k]-m_fluxes[i][k]);
#endif
                restore(m_cells[i],m_states[i]);
                // Darcy wall friction; lost bulk energy remains as gas heat.
                const double oldBulk=m_cells[i].bulkKineticEnergy();
                m_cells[i].m_state.momentum[0]/=1+m_frictionFactor*std::abs(m_cells[i].velocity_x())*h/(2*diameter);
                m_cells[i].changeEnergy(oldBulk-m_cells[i].bulkKineticEnergy());
            }
            dt-=h;
        }
    }

    static void advancePair(GasPipe &a, GasPipe &b, double dt) {
        GasPipe *pipes[2]={&a,&b};
        advanceBatch(pipes,2,dt);
    }
    static void advanceBatch(GasPipe *const *pipes, int count, double dt) {
        ENGINE_SIM_PROFILE_SCOPE(Pipes);
        if (!gpu_pipe::enabled()) {
            for(int j=0;j<count;++j) pipes[j]->advance(dt);
            return;
        }
        thread_local std::vector<gpu_pipe::Pipe> batch;
        thread_local std::vector<GasPipe *> targets;
        // Reuse records: clearing/emplacing zeroed all 64 slots for every pipe
        // even when only eight cells were active.
        batch.resize(count); targets.clear();
        int gpuCount=0;
        for(int j=0;j<count;++j) {
            auto &pipe=*pipes[j];
            if(!pipe.active()) continue;
            if(!pipe.first().m_variableProperties) {pipe.advance(dt); continue;}
            targets.push_back(&pipe);
            auto &out=batch[gpuCount++];
            out.count=pipe.count(); out.dx=pipe.m_dx;
            out.diameter=2*std::sqrt(pipe.m_area/constants::pi);
            out.friction=pipe.m_frictionFactor;
            out.fuelMass=pipe.m_cells.front().mix().fuelMolecularMass;
            for (int i=0;i<out.count;++i) {
                const auto u=conserved(pipe.m_cells[i]);
                std::copy(u.begin(),u.end(),out.u[i]);
            }
        }
        if(gpuCount==0) return;
        gpu_pipe::advance(batch.data(),gpuCount,dt);
        for(int j=0;j<gpuCount;++j) {
            auto &pipe=*targets[j];
            pipe.m_cachedCflStates.resize(pipe.m_cells.size());
            for(int i=0;i<batch[j].count;++i) {
                Vector u; std::copy(batch[j].u[i],batch[j].u[i]+8,u.begin());
                restore(pipe.m_cells[i],u);
                std::memcpy(&pipe.m_cachedCflStates[i],&pipe.m_cells[i].m_state,sizeof(GasSystem::State));
            }
            pipe.m_cachedCflStep=batch[j].stableTimestep;
            pipe.m_cachedCflValid=true;
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
        Vector u{}; GasTransport::conserved(g,u.data()); return u;
    }
    // Rusanov interface flux: two advective fluxes (u*v with the pressure
    // terms folded in exactly where flux() added them) plus the dissipative
    // state difference. v/p come from the per-substep scalar precompute.
    static Vector combineFlux(const Vector &uL, double vL, double pL,
            const Vector &uR, double vR, double pR, double a) {
        Vector fl{}, fr{}, f{};
        for (int k=0;k<8;++k) { fl[k]=uL[k]*vL; fr[k]=uR[k]*vR; }
        fl[5]+=pL; fl[6]+=pL*vL;
        fr[5]+=pR; fr[6]+=pR*vR;
#if defined(__AVX2__)
        const __m256d half=_mm256_set1_pd(0.5);
        const __m256d halfA=_mm256_set1_pd(0.5*a);
        const __m256d sum=_mm256_mul_pd(_mm256_add_pd(
            _mm256_loadu_pd(fl.data()),_mm256_loadu_pd(fr.data())),half);
        const __m256d sum4=_mm256_mul_pd(_mm256_add_pd(
            _mm256_loadu_pd(fl.data()+4),_mm256_loadu_pd(fr.data()+4)),half);
        const __m256d diff=_mm256_mul_pd(halfA,_mm256_sub_pd(
            _mm256_loadu_pd(uR.data()),_mm256_loadu_pd(uL.data())));
        const __m256d diff4=_mm256_mul_pd(halfA,_mm256_sub_pd(
            _mm256_loadu_pd(uR.data()+4),_mm256_loadu_pd(uL.data()+4)));
        _mm256_storeu_pd(f.data(),_mm256_sub_pd(sum,diff));
        _mm256_storeu_pd(f.data()+4,_mm256_sub_pd(sum4,diff4));
#else
        for (int k=0;k<8;++k)
            f[k]=0.5*(fl[k]+fr[k])-0.5*a*(uR[k]-uL[k]);
#endif
        return f;
    }
    static void restore(GasSystem &g,const Vector &u) {
        if(GasTransport::restore(g,u.data())) throw std::runtime_error("Pipe state positivity failure");
    }
    std::vector<GasSystem> m_cells;
    std::vector<GasSystem::State> m_cachedCflStates;
    mutable bool m_cachedCflValid=false;
    double m_cachedCflStep=0;
    std::vector<Vector> m_fluxes,m_states;
    std::vector<double> m_v,m_p,m_s;
    double m_area=1,m_dx=1,m_frictionFactor=0.02;
};
#endif
