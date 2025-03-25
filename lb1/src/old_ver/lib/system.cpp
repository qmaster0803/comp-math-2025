#include "../include/system.h"
#include <exception>
#include <stdexcept>

void System::operator()(const system_state_t &x, system_state_t &dxdt, double t)
{
    if(x.size() != bricks.size()*13)
        throw std::invalid_argument("Wrong system state length!");
        
    for(auto &i : bricks) {
        // refresh computed variables
        // brick.update_from_array(x);
        // brick.compute_derived_variables(t);
        // brick.compute_force_and_torque(t);

        // get dxdt of the brick
        // dxdt = brick.dxdt(t);
    }
}  


void System::observer(const system_state_t &x, double t) const
{
    if(x.size() != bricks.size()*13)
        throw std::invalid_argument("Wrong system state length!");

    // brick.update_from_array(x);

    // glm::vec3 rotation_metric(1, 0, 0);
    // rotation_metric = rotation_metric * glm::mat3_cast(brick.orientation);
    
    // std::cout << "Observed at t=" << t << ": position="
    //           << "(" << brick.position.x << ", " << brick.position.y << ", " << brick.position.z << ")"
    //           << "rotation="
    //           << "(" << rotation_metric.x << ", " << rotation_metric.y << ", " << rotation_metric.z << ")"
    //           << std::endl;
}

system_state_t System::get_state_as_array() const
{
    system_state_t output;
    for(auto &brick : bricks) {
        auto tmp = brick.get_state_as_array();
        output.insert(output.end(), tmp.begin(), tmp.end());
    }
    return output;
}
