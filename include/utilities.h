#ifndef ATG_ENGINE_SIM_UTILITIES_H
#define ATG_ENGINE_SIM_UTILITIES_H
#include "gas_execution.h"

double modularDistance(double a, double b, double mod = 1.0);
double positiveMod(double x, double mod);
double erfApproximation(double x);

template <typename t>
ES_GAS_FUNCTION inline t clamp(t x, t x0 = static_cast<t>(0.0), t x1 = static_cast<t>(1.0)) {
    if (x <= x0) return x0;
    else if (x >= x1) return x1;
    else return x;
}

#endif /* ATG_ENGINE_SIM_UTILITIES_H */
