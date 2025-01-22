#ifndef FORWARDEULER_H
#define FORWARDEULER_H

#include "ExplicitIntegrator.h"

class ForwardEuler : public ExplicitIntegrator {
    ForwardEuler(){};
    ~ForwardEuler(){};
    Eigen::VectorXd integrate_system_dynamics(
        Eigen::VectorXd x0, Eigen::VectorXd u0,
        const std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd, double)> &dx_dt, 
        double t0, double tf);
};

Eigen::VectorXd ForwardEuler::integrate_system_dynamics(
        Eigen::VectorXd x0, Eigen::VectorXd u0,
        const std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd, double)> &dx_dt, 
        double t0, double tf){
                // Initialize final state
                Eigen::VectorXd::Zero(x0.size()) xf; 
                
                // Forward euler equation 
                xf = dx_dt(x0, u0, t0) * (tf - t0) + x0;
                
                return xf;
        }

#endif
