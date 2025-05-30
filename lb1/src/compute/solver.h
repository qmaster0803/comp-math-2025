#pragma once
#include <stdlib.h>
#include <array>
#include <glm/mat3x4.hpp>

namespace solver
{
    template<typename T>
    concept HasSolvingMethods = requires(T t, const std::array<double, 13> i) {
        { t.update_from_array(i) } -> std::same_as<void>;
        { t.state_as_array() }     -> std::same_as<std::array<double, 13>>;
        { t.dxdt() }               -> std::same_as<std::array<double, 13>>;
    };

    template<HasSolvingMethods T>
    void euler_solver(T &object, double t_to_sim, double step = 0.05);

    template<HasSolvingMethods T>
    void rk4_solver(T &object, double t_to_sim);

    template<HasSolvingMethods T>
    void rk5_solver(T &object, double t_to_sim);

    template<std::size_t N>
    void sum_arrays(std::array<double, N> &dest, const std::array<double, N> &src);
    
    template<std::size_t N>
    void mul_array(std::array<double, N> &dest, double k);

    bool check_value_greater(double v1, double v2, double epsilon);
    bool check_value_less(double v1, double v2, double epsilon);
    bool check_value_equal(double v1, double v2, double epsilon);

    void reduce_to_RREF(glm::dmat3x4& matrix);
}
