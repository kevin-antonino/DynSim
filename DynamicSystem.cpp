#include "DynamicSystem.h"
#include <stdexcept>
// TODO / Bugs: Input_systems is not being changed in connect_input, and lines 61 and 65 need to use time_stamp instead of future time
DynamicSystem::DynamicSystem(int n_states, int n_inputs, int n_outputs){
    // Allocate memory
    state.resize(n_states);
    derivative.resize(n_states);
    input.resize(n_inputs);
    output.resize(n_outputs);
    
    // Initialize the system at the origin
    initialize(Eigen::VectorXd::Zero(n_states));
}

void DynamicSystem::initialize(Eigen::VectorXd init_state, double init_time) : time_stamp(init_time) {
    if ( init_state.size() != state.size() ){
        throw std::runtime_error("Number of states do not match: state size = " + std::to_string(state.size()) + ", initial state size = " + std::to_string(init_state.size()));
    }
    // Set state to initial value
    state = init_state;

    // Notify output systems that the system has been initialized
    notify_output_systems();
}

void DynamicSystem::propagate_to(double future_time){
    if (time_stamp < future_time){ 
        // 1. Advance state by integrating to future time
        state = integrator.integrate_system_dynamics(state, input, &dynamic_equation, time_stamp, future_time);
        time_stamp = future_time;

        // 2. Notify output systems that the state and time stamp have been updated
        notify_output_systems();
    }else if(time_stamp > future_time){
        // Future time is not valid
        throw std::runtime_error("Cannot propagate to an earlier time: system time = " + std::to_string(time_stamp) + ", requested = " + std::to_string(future_time));
    }
    // time_stamp == future_time: does nothing
}

void DynamicSystem::notify_output_systems(){
    if (!output_systems.empty()){
        for (DynamicSystem* sys : output_systems)
            sys->await_and_update(this);
    }
}

void DynamicSystem::await_and_update(DynamicSystem* in_sys){
    if (in_sys->get_time_stamp() != this->time_stamp){
        throw std::runtime_error("Time stamps do not match. Input system = " + std::to_string(in_sys->get_time_stamp()) + ", This system = " + std::to_string(time_stamp));
    }
    // 1. Await input systems' update messages
    updated_systems.insert(in_sys); // Add to list of updated systems and wait

    // 2. Update if all are up-to-date
    if (updated_systems.size() == input_systems.size() && updated_systems == input_systems){
        // Advance inputs (if any)
        input = input_bus.get_values();

        // Advance derivative 
        derivative = dynamic_equation(state, input, future_time);

        // Advance outputs
        output = output_equation(state, input, future_time);
    }
}

void DynamicSystem::connect_input(DynamicSystem* input_system, const std::array<int> & output_indeces){
    // 1. Register this system as an output system (subject) for the input system (observer)
    input_system->add_output_system(this);

    // 2. Connect the input system to input_bus to pull inputs
    for(int ii=0; ii < output_indeces.size(); ii++){
        if (output_indeces[ii] < input_system->output.size()){
            double* output_ptr = input_system->output.data() + output_indeces[ii];
            input_bus.connect_system_output(output_ptr);
        } else {
            throw std::runtime_error("Output index = " + std::to_string(output_indeces[ii]) + " is out of bounds of output size = " + std::to_string(input_system->output.data()))
        }
    }
}

void DynamicSystem::add_output_system(DynamicSystem* out_sys){
    output_systems.insert(out_sys);
}   

double DynamicSystem::get_time_stamp() const {
    return time_stamp;
}
