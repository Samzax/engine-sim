#ifndef ATG_ENGINE_SIM_GAS_THERMO_H
#define ATG_ENGINE_SIM_GAS_THERMO_H

#include "constants.h"
#include <algorithm>

namespace gas_thermo {
// NASA7 cp/R coefficients, Cantera data/nasa_gas.yaml (NASA thermodynamic data).
// Species order: N2, O2, CO2, H2O, isooctane. 200..1000 and 1000..6000 K.
inline constexpr double coefficients[5][2][5] = {
    {{3.53100528,-1.23660987e-4,-5.02999437e-7,2.43530612e-9,-1.40881235e-12},
     {2.95257626,1.39690057e-3,-4.92631691e-7,7.86010367e-11,-4.60755321e-15}},
    {{3.78245636,-2.99673415e-3,9.847302e-6,-9.68129508e-9,3.24372836e-12},
     {3.66096083,6.56365523e-4,-1.41149485e-7,2.05797658e-11,-1.29913248e-15}},
    {{2.35677352,8.98459677e-3,-7.12356269e-6,2.45919022e-9,-1.43699548e-13},
     {4.63659493,2.74131991e-3,-9.95828531e-7,1.60373011e-10,-9.16103468e-15}},
    {{4.19864056,-2.0364341e-3,6.52040211e-6,-5.48797062e-9,1.77197817e-12},
     {2.67703787,2.97318329e-3,-7.7376969e-7,9.44336689e-11,-4.26900959e-15}},
    {{0.815737338,0.0732643959,1.78300688e-5,-6.9358962e-8,3.21629382e-11},
     {15.9899273,0.055318479,-1.95267072e-5,3.11779172e-9,-1.85312577e-13}}
};

inline double cvPolynomial(const double *a, double t) {
    return constants::R * ((((a[4]*t+a[3])*t+a[2])*t+a[1])*t+a[0]-1);
}
inline double integral(const double *a, double t) {
    return constants::R * t * ((((a[4]*t/5+a[3]/4)*t+a[2]/3)*t+a[1]/2)*t+a[0]-1);
}
inline double cv(int species, double t) {
    t = std::clamp(t, 200.0, 6000.0);
    return cvPolynomial(coefficients[species][t <= 1000 ? 0 : 1], t);
}
// Continuous sensible internal energy with u(0)=0. Hold endpoint cv outside
// the data interval instead of extrapolating the polynomial to negative cv.
inline double u(int species, double t) {
    const auto &a = coefficients[species];
    const double low = cv(species, 200) * 200;
    if (t <= 200) return cv(species, 200) * t;
    const double middle = low + integral(a[0], 1000) - integral(a[0], 200);
    if (t <= 1000) return low + integral(a[0], t) - integral(a[0], 200);
    const double high = middle + integral(a[1], 6000) - integral(a[1], 1000);
    if (t <= 6000) return middle + integral(a[1], t) - integral(a[1], 1000);
    return high + cv(species, 6000) * (t - 6000);
}
}
#endif
