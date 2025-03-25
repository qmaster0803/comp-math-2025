#pragma once

// Class T must have dxdt(double t) method that returns some iterable
// that supports operator[] and has the same length as x
// Also class T must support operator[] and std::size_t size()

template<class T>
void euler_ODE_solver(T &x, double t0, double t1, double t_step)
{
    
}
