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
template void solver::sum_arrays<13>(std::array<double, 13>&, const std::array<double, 13>&);
template void solver::mul_array<13>(std::array<double, 13>&, double);
