#pragma once
#include <vector>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

struct Brick_size
{
    unsigned x_size, y_size, z_size, mass;
    Brick_size(unsigned x_size, unsigned y_size, unsigned z_size, unsigned mass)
        : x_size{x_size}, y_size{y_size}, z_size{z_size}, mass{mass} {}
};

// Why should I've called it other way?
class Brick {
public:
    // State variables
    glm::dvec3 position;
    glm::dquat orientation = glm::dquat(1, 0, 0, 0);
    
    glm::dvec3 linear_momentum;
    glm::dvec3 angular_momentum;
    
    // Constants
    glm::dmat3x3 body_inertia_tensor;
    glm::dmat3x3 body_inertia_tensor_inv;

    const Brick_size dimensions;

    // Derived (computed) variables
    glm::dmat3x3 orientation_matrix;
    glm::dvec3   velocity;
    glm::dvec3   angular_velocity;
    glm::dvec3   current_force;
    glm::dvec3   current_torque;

    // Default constructor
    Brick(Brick_size dimensions);

    // Compute force and torque acting on the body at time t
    void compute_force_and_torque(double t);

    void compute_derived_variables(double t);

    // Compute body state derivative at time t
    std::vector<double> dxdt(double t) const;

    // Update body state from array
    void update_from_array(std::vector<double> st);
    std::vector<double> get_state_as_array() const;
};
