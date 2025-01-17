#ifndef FORWARDEULER_H
#define FORWARDEULER_H

#include "ExplicitIntegrator.h"

class ForwardEuler : public ExplicitIntegrator {
    ForwardEuler(){};
    ~ForwardEuler(){};
    std::vector<double> advance_by_dt(
        std::vector<double> x0, 
        const std::function<std::vector<double>(std::vector<double>, std::vector<double>, double)> &dx_dt, 
        double dt); // input: initial state, derivative function, time step of integration. Output: x(t+dt)
};

std::vector<double>  ForwardEuler::advance_by_dt(
        std::vector<double> x0, 
        const std::function<std::vector<double>(std::vector<double>, std::vector<double>, double)> &dx_dt, 
        double dt){

        }

#endif
