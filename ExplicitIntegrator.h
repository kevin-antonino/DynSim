#ifndef EXPLICITINTEGRATOR_H
#define EXPLICITINTEGRATOR_H

#include<vector>
#include<function>

class ExplicitIntegrator{
public:
    ExplicitIntegrator(){};
    ~ExplicitIntegrator(){};
    virtual std::vector<double> advance_by_dt(
        std::vector<double> x0, 
        const std::function<std::vector<double>(std::vector<double>, std::vector<double>, double)> &dx_dt, 
        double t0, 
        double tf) = 0; // input: initial state, derivative function, time range of integration. Output: x(t+dt)
};

#endif
