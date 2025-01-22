#ifndef EXPLICITINTEGRATOR_H
#define EXPLICITINTEGRATOR_H

#include<vector>
#include<function>

class ExplicitIntegrator{
// note to possibly remove u0 as an argument and force the user of the class to have evaluated any inputs before passing dx_dt
// note to add dt property and ability to integrate within sim time steps 
public:
    ExplicitIntegrator(){};
    ~ExplicitIntegrator(){};
    virtual Eigen::VectorXd integrate_system_dynamics(
        Eigen::VectorXd x0, Eigen::VectorXd u0,
        const std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd, double)> &dx_dt, 
        double t0, double tf) = 0; // input: initial state, derivative function, time range of integration. Output: x(t+dt)
};

#endif
