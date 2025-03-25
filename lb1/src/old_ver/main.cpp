#include <ios>
#include <iostream>
#include <boost/numeric/odeint.hpp>
#include "include/brick.h"
#include "include/system.h"






int main()
{   
    std::vector<Brick_size> brick_sizes;
    brick_sizes.emplace_back(1, 1, 1, 10);
    System system(brick_sizes);
    
    boost::numeric::odeint::runge_kutta4<system_state_t> stepper;
    system_state_t x0 = system.get_state_as_array();

    std::cout << std::fixed;
    std::cout.width(10);
    std::cout.precision(3);

    boost::numeric::odeint::integrate_const(stepper, system, x0, 0.0, 5.0, 0.1);
    return 0;
}
