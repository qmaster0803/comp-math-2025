#include "solver.h"
#include <cmath>

template<solver::HasSolvingMethods T>
void solver::euler_solver(T &object, double t_to_sim, double step)
{
    auto current_state = object.state_as_array();
    double t_elapsed = 0;
    while(1) {
        auto current_dxdt = object.dxdt();
        double dt = std::fmin(t_to_sim - t_elapsed, step);
        solver::mul_array(current_dxdt, dt);
        solver::sum_arrays(current_state, current_dxdt);
        object.update_from_array(current_state);

        if(t_to_sim - t_elapsed < step)
            break;
        else
            t_elapsed += dt;
    }
}

template<solver::HasSolvingMethods T>
void solver::rk4_solver(T &object, double t_to_sim)
{
    auto initial_state = object.state_as_array();
    
    // Compute k1 (derivative at 0)
    auto k1 = object.dxdt();
    auto state_k1 = initial_state;
    solver::mul_array(k1, t_to_sim * 0.5); // k1 = h*k1/2
    solver::sum_arrays(state_k1, k1);      // state_k1 = initial_state + h*k1/2
    
    // Compute k2 (derivative at h*k1 / 2)
    object.update_from_array(state_k1);
    auto k2 = object.dxdt();
    auto state_k2 = initial_state;
    solver::mul_array(k2, t_to_sim * 0.5); // k2 = h*k2/2
    solver::sum_arrays(state_k2, k2);      // state_k2 = initial_state + h*k2/2

    // Compute k3 (derivative at h*k2 / 2)
    object.update_from_array(state_k2);
    auto k3 = object.dxdt();
    auto state_k3 = initial_state;
    solver::mul_array(k3, t_to_sim);       // k3 = h*k3
    solver::sum_arrays(state_k2, k2);      // state_k3 = initial_state + h*k3

    // Compute k4 (derivative at h*k3)
    object.update_from_array(state_k3);
    auto k4 = object.dxdt();
    solver::mul_array(k4, t_to_sim);       // k4 = h*k4

    // Compute result
    // (h / 6) * (k1 + 2*k2 + 2*k3 + k4) = (1/6) * (h*k1 + h*2*k2 + h*2*k3 + h*k4)
    solver::mul_array(k1, 2.0); // k1 = h*k1
    solver::mul_array(k2, 4.0); // k2 = h*2*k2
    solver::mul_array(k3, 2.0); // k3 = h*2*k3

    solver::sum_arrays(k1, k2);
    solver::sum_arrays(k1, k3);
    solver::sum_arrays(k1, k4);

    solver::mul_array(k1, 1.0 / 6.0);
    solver::sum_arrays(initial_state, k1);

    object.update_from_array(initial_state);
}

template<std::size_t N>
void solver::sum_arrays(std::array<double, N> &dest, const std::array<double, N> &src)
{
    for(std::size_t i = 0; i < dest.size(); i++) {
        dest[i] += src[i];
    }
}

template<std::size_t N>
void solver::mul_array(std::array<double, N> &dest, double k)
{
    for(std::size_t i = 0; i < dest.size(); i++) {
        dest[i] *= k;
    }
}

bool solver::check_value_greater(double v1, double v2, double epsilon)
{
    double res = v1 - v2;
    return res > epsilon;
}

bool solver::check_value_less(double v1, double v2, double epsilon)
{
    double res = v2 - v1;
    return res > epsilon;
}

bool solver::check_value_equal(double v1, double v2, double epsilon)
{
    double res = v1 - v2;
    return abs(res) <= epsilon;
}

void solver::reduce_to_RREF(glm::dmat3x4 &matrix) {
    int lead = 0;
    int rowCount = matrix.length();
    int colCount = matrix[0].length();

    for (int r = 0; r < rowCount; r++) {
        if (lead >= colCount) break;

        int i = r;
        while (matrix[i][lead] == 0) {
            i++;
            if (i == rowCount) {
                i = r;
                lead++;
                if (lead == colCount) return;
            }
        }

        if (i != r) {
            for (int k = 0; k < colCount; k++) {
                std::swap(matrix[i][k], matrix[r][k]);
            }
        }

        float val = matrix[r][lead];
        for (int k = 0; k < colCount; k++) {
            matrix[r][k] /= val;
        }

        for (int i = 0; i < rowCount; i++) {
            if (i != r) {
                float factor = matrix[i][lead];
                for (int k = 0; k < colCount; k++) {
                    matrix[i][k] -= factor * matrix[r][k];
                }
            }
        }

        lead++;
    }
}

// Explicit instantiation to compile function templates
#include "../model/cube.h"
template void solver::euler_solver<Cube>(Cube&, double, double);
template void solver::rk4_solver<Cube>(Cube&, double);
template void solver::sum_arrays<13>(std::array<double, 13>&, const std::array<double, 13>&);
template void solver::mul_array<13>(std::array<double, 13>&, double);
