#include<vector>

class SystemBus
{
private:
    std::vector<const double*> bus_value_ptrs
public:
    connect_system_output(double& output_ptr)
    Eigen::VectorXd get_bus_values() const;
};

void SystemBus::connect_system_output(double& output_ptr){
    bus_value_ptrs.push_back(output_ptr);
}

Eigen::VectorXd SystemBus::get_values() const {
    if (bus_value_ptrs.size() > 0){
        Eigen::VectorXd bus_values(bus_value_ptrs.size());

        for (int ii = 0; ii < bus_values.size(); ii++){
            bus_values[ii] = bus_value_ptrs[ii]*;
        }
        return bus_values;
    } else {
        // return not-a-number if there's no inputs
        return std::nan;
    }
}
