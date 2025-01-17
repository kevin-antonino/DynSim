#ifndef DYNAMICSYSTEM_H
#define DYNAMICSYSTEM_H
#include<vector>
#include<list>
#include "ExplicitIntegrator.h"
#include <Eigen/Core>

class DynamicSystem
{
private:
    // Inter-system communication for observer / subject interaction
    void notify_output_systems();                              // Notifies all dependent systems that this system's output is ready to be pulled
    void await_and_update(DynamicSystem* in_sys);              // Records updated input systems and if all are updated, will pull input from bus
    void add_output_system(DynamicSystem* out_sys);            // Adds this system's output into another system's input bus
    std::set<DynamicSystem*> updated_systems;                  // List of input systems that have been updated to this system's time_stamp
    std::set<DynamicSystem*> input_systems;                   
    std::set<DynamicSystem*> output_systems;        

protected:
    // State, derivative, input, and output variables
    Eigen::VectorXd state;                          // x(t)
    Eigen::VectorXd derivative;                     // xdot(t)
    Eigen::VectorXd input;                          // u(t)
    Eigen::VectorXd output;                         // y(t)

    double time_stamp;                              // time that state, derivative, output, and input are valid for
    ExplicitIntegrator integrator;                  // integration method for evolving the state in time
    SystemBus input_bus;

    // Virtual dynamic and output equations
    virtual Eigen::VectorXd const dynamic_equation(Eigen::VectorXd x, Eigen::VectorXd u, double t) = 0;  // xdot = f(x, u, t)
    virtual Eigen::VectorXd const output_equation(Eigen::VectorXd x, Eigen::VectorXd u, double t) = 0;   // y = g(x, u, t)    

public:
    // Parameterized constuctor with number of elements for x, xdot, y, u
    DynamicSystem(int n_states, int n_inputs, int n_outputs);

    // Core methods for simulation
    void initialize(Eigen::VectorXd init_state, double init_time = 0);
    void propagate_to(double future_time);
    void connect_input(DynamicSystem* input_system, const std::array<int> output_indeces);

    // Getter methods
    double DynamicSystem::get_time_stamp() const;

    // Dynamic Systems are friends so they can use the private communication attributes/methods
    friend class DynamicSystem;
};

#endif
