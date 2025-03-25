#include "../include/brick.h"

Brick::Brick(unsigned x_size, unsigned y_size, unsigned z_size, float mass)
    : x_size{x_size}, y_size{y_size}, z_size{z_size}, mass{mass}
{
    // Compute I_body
    // Because shape of the body is very simple there're no integrals.
    body_inertia_tensor = glm::dmat3x3((mass/12)*(y_size*y_size + z_size*z_size), 0, 0,
                                       0, (mass/12)*(x_size*x_size + z_size*z_size), 0,
                                       0, 0, (mass/12)*(x_size*x_size + y_size*y_size));
    body_inertia_tensor_inv = glm::inverse(body_inertia_tensor);
}

void Brick::compute_force_and_torque(double t)
{
    // debug filler: apply force for one second
    current_force = (1 <= t && t <= 2) ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 0, 0);
    current_torque = (1 <= t && t <= 2) ? glm::dvec3(0, 0, 1) : glm::dvec3(0, 0, 0);
    
}

void Brick::compute_derived_variables(double t)
{
    orientation_matrix = glm::mat3_cast(orientation);
    velocity = linear_momentum / mass;
    
    glm::mat3x3 current_inertia_tensor_inv =
        orientation_matrix * body_inertia_tensor_inv * glm::transpose(orientation_matrix);
    angular_velocity = current_inertia_tensor_inv * angular_momentum;
}

std::array<double, 13> Brick::dxdt(double t)
{
    std::array<double, 13> output;

    // position derivative is linear velocity
    output[0] = this->velocity.x;
    output[1] = this->velocity.y;
    output[2] = this->velocity.z;

    // orientation derivate is angular velocity
    glm::dquat tmp(0, this->angular_velocity);
    glm::dquat orientation_deriv = 0.5 * (tmp * this->orientation);
    output[3] = orientation_deriv.x;
    output[4] = orientation_deriv.y;
    output[5] = orientation_deriv.z;
    output[6] = orientation_deriv.w;

    // linear momentum derivative is applied force
    output[7] = this->current_force.x;
    output[8] = this->current_force.y;
    output[9] = this->current_force.z;

    // angular momentum derivative is applied torque
    output[10] = this->current_torque.x;
    output[11] = this->current_torque.y;
    output[12] = this->current_torque.z;

    return output;
}

void Brick::update_from_array(std::array<double, 13> st)
{
    this->position.x = st[0];
    this->position.y = st[1];
    this->position.z = st[2];

    this->orientation.x = st[3];
    this->orientation.y = st[4];
    this->orientation.z = st[5];
    this->orientation.w = st[6];

    this->linear_momentum.x = st[7];
    this->linear_momentum.y = st[8];
    this->linear_momentum.z = st[9];

    this->angular_momentum.x = st[10];
    this->angular_momentum.y = st[11];
    this->angular_momentum.z = st[12];
}

std::array<double, 13> Brick::get_state_as_array()
{
    std::array<double, 13> output;

    output[0] = this->position.x;
    output[1] = this->position.y;
    output[2] = this->position.z;

    output[3] = this->orientation.x;
    output[4] = this->orientation.y;
    output[5] = this->orientation.z;
    output[6] = this->orientation.w;

    output[7] = this->linear_momentum.x;
    output[8] = this->linear_momentum.y;
    output[9] = this->linear_momentum.z;

    output[10] = this->angular_momentum.x;
    output[11] = this->angular_momentum.y;
    output[12] = this->angular_momentum.z;

    return output;
}
