#ifndef ENGINE_SIM_GAS_TRANSPORT_H
#define ENGINE_SIM_GAS_TRANSPORT_H
#include "gas_system.h"
#include <algorithm>

struct GasTransport {
    ES_GAS_FUNCTION static bool finite(double x) { return x>=-DBL_MAX && x<=DBL_MAX; }
    ES_GAS_FUNCTION static void conserved(const GasSystem &g,double *u) {
        const auto m=g.mix(); const double c=g.n()/g.volume();
        u[0]=c*m.p_fuel; u[1]=c*m.p_o2; u[2]=c*(m.p_inert-m.p_co2-m.p_h2o);
        u[3]=c*m.p_co2; u[4]=c*m.p_h2o;
        u[5]=g.m_state.momentum[0]/g.volume(); u[6]=g.totalEnergy()/g.volume();
        u[7]=g.mass()*m.residualFraction/g.volume();
    }
    ES_GAS_FUNCTION static int restore(GasSystem &g,const double *u) {
        double molarDensity=0;
        for (int k=0;k<5;++k) {
            if (!finite(u[k]) || u[k]<-1e-10)
                return 6;
            molarDensity+=(std::max)(0.0,u[k]);
        }
        if (!(molarDensity>0)) return 6;
        auto &m=g.m_state.mix;
        m.p_fuel=(std::max)(0.0,u[0])/molarDensity;
        m.p_o2=(std::max)(0.0,u[1])/molarDensity;
        m.p_co2=(std::max)(0.0,u[3])/molarDensity;
        m.p_h2o=(std::max)(0.0,u[4])/molarDensity;
        m.p_inert=((std::max)(0.0,u[2])+(std::max)(0.0,u[3])+(std::max)(0.0,u[4]))/molarDensity;
        g.m_state.n_mol=molarDensity*g.volume();
        g.m_state.momentum[0]=u[5]*g.volume(); g.m_state.momentum[1]=0;
        g.invalidateProperties(); g.m_cachedEnergy=-1;
        m.residualFraction=(std::min)((std::max)(u[7]*g.volume()/g.mass(),0.0),1.0);
        g.m_state.E_k=u[6]*g.volume()-g.bulkKineticEnergy();
        if (!finite(g.m_state.E_k) || g.m_state.E_k<=0)
            return 6;
        return 0;
    }
};
#endif
