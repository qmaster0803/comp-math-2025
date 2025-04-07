#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include "cube.h"
#include "../compute/solver.h"

Cube::Cube(glm::dvec3 initial_position, glm::dvec3 size, double mass) :
_position{initial_position}, size{size}, mass{mass} {
    _cube_mesh = new CubeMesh(size);
    // Compute I_body
    // Because shape of the body is very simple there're no integrals.
    _body_inertia_tensor = glm::dmat3x3((mass/12)*(size.y*size.y + size.z*size.z), 0, 0,
                                        0, (mass/12)*(size.x*size.x + size.z*size.z), 0,
                                        0, 0, (mass/12)*(size.x*size.x + size.y*size.y));
    _body_inertia_tensor_inv = glm::inverse(_body_inertia_tensor);
    
    _compute_derived_variables();
}

void Cube::set_force_and_torque(glm::dvec3 force, glm::dvec3 torque)
{
    _current_force  = force;
    _current_torque = torque;
}

#include <iostream>
void Cube::apply_impulse(const glm::dvec3 &linear, const glm::dvec3 &angular)
{
    std::cout << "Before change: " << std::endl;
    std::cout << "_linear_momentum: (" << _linear_momentum.x << "; " << _linear_momentum.y << "; " << _linear_momentum.z << ")" << std::endl;
    std::cout << "_angular_momentum: (" << _angular_momentum.x << "; " << _angular_momentum.y << "; " << _angular_momentum.z << ")" << std::endl;
    std::cout << "_velocity: (" << _velocity.x << "; " << _velocity.y << "; " << _velocity.z << ")" << std::endl;
    _linear_momentum += linear;
    _angular_momentum += angular;
    _compute_derived_variables();
    std::cout << "After change: " << std::endl;
    std::cout << "_linear_momentum: (" << _linear_momentum.x << "; " << _linear_momentum.y << "; " << _linear_momentum.z << ")" << std::endl;
    std::cout << "_velocity: (" << _velocity.x << "; " << _velocity.y << "; " << _velocity.z << ")" << std::endl;
    
}

glm::mat4 Cube::get_transform() const
{
    glm::mat4 transformation_matrix = glm::translate(glm::mat4(1.0f), glm::vec3(_position));
    transformation_matrix *= glm::mat4(_orientation_matrix);
    return transformation_matrix;
}

unsigned Cube::get_cube_mesh() const
{
    return _cube_mesh->get_vao();
}

void Cube::_compute_derived_variables()
{
    _orientation = glm::normalize(_orientation);
    _orientation_matrix = glm::mat3_cast(_orientation);
    _velocity = _linear_momentum / mass;
    
    glm::mat3x3 current_inertia_tensor_inv =
        _orientation_matrix * _body_inertia_tensor_inv * glm::transpose(_orientation_matrix);
    _angular_velocity = current_inertia_tensor_inv * _angular_momentum;
}

std::array<glm::dvec4, 6> Cube::get_faces() const
{
    // U, L, F, R, B, D
    std::array<glm::dvec3, 6> normals;
    normals[0] = glm::dvec3( 0.0,  0.0,  1.0);  // Up
    normals[1] = glm::dvec3(-1.0,  0.0,  0.0);  // Left
    normals[2] = glm::dvec3( 0.0, -1.0,  0.0);  // Front
    normals[3] = glm::dvec3( 1.0,  0.0,  0.0);  // Right
    normals[4] = glm::dvec3( 0.0,  1.0,  0.0);  // Back    
    normals[5] = glm::dvec3( 0.0,  0.0, -1.0);  // Down

    for(auto &i : normals) {
        i = _orientation_matrix * i;
    }
    // now we have correct plane normals in the array, let's find D for every plane

    auto lambda_find_D = [](glm::dvec3 normal, glm::dvec3 vertex) {
        return glm::dot(normal, vertex) * -1.0;
    };
    
    glm::dvec3 up_left_back_vertex = glm::dvec3(size.x/-2.0,
                                                size.y/2.0,
                                                size.z/2.0);
    
    glm::dvec3 down_right_front_vertex = glm::dvec3(size.x/2.0,
                                                    size.y/-2.0,
                                                    size.z/-2.0);
    up_left_back_vertex     = (_orientation_matrix * up_left_back_vertex    ) + _position;
    down_right_front_vertex = (_orientation_matrix * down_right_front_vertex) + _position;

    std::array<glm::dvec4, 6> result;
    result[0] = glm::dvec4(normals[0], lambda_find_D(normals[0], up_left_back_vertex));
    result[1] = glm::dvec4(normals[1], lambda_find_D(normals[1], up_left_back_vertex));
    result[2] = glm::dvec4(normals[2], lambda_find_D(normals[2], down_right_front_vertex));
    result[3] = glm::dvec4(normals[3], lambda_find_D(normals[3], down_right_front_vertex));
    result[4] = glm::dvec4(normals[4], lambda_find_D(normals[4], up_left_back_vertex));
    result[5] = glm::dvec4(normals[5], lambda_find_D(normals[5], down_right_front_vertex));

    return result;
}

// 0---1  4---5
// | U |  | D |
// 2---3  6---7
std::array<glm::dvec3, 8> Cube::get_vertices() const
{
    double x2 = size.x/2;
    double y2 = size.y/2;
    double z2 = size.z/2;
    std::array<glm::dvec3, 8> result;
    result[0] = glm::dvec3(-x2,  y2, z2);
    result[1] = glm::dvec3( x2,  y2, z2);
    result[2] = glm::dvec3(-x2, -y2, z2);
    result[3] = glm::dvec3( x2, -y2, z2);
    
    result[4] = glm::dvec3(-x2,  y2, -z2);
    result[5] = glm::dvec3( x2,  y2, -z2);
    result[6] = glm::dvec3(-x2, -y2, -z2);
    result[7] = glm::dvec3( x2, -y2, -z2);

    for(auto &i : result) {
        i = _orientation_matrix * i;
        i += _position;
    }

    return result;
}

bool Cube::check_point_on_surface(glm::dvec3 point) const
{
    // convert point to local coordinate system
    point = glm::inverse(_orientation_matrix) * (point - _position);

    // check whether point belongs to the surface
    return (solver::check_value_equal(abs(point.x), size.x/2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(point.y,    size.y/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(point.y, size.y/-2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(point.z,    size.z/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(point.z, size.z/-2.0, SURFACE_POINT_CHECK_TOLERANCE)) ||
    
           (solver::check_value_equal(abs(point.y), size.y/2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(point.x,    size.x/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(point.x, size.x/-2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(point.z,    size.z/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(point.z, size.z/-2.0, SURFACE_POINT_CHECK_TOLERANCE)) ||
           
           (solver::check_value_equal(abs(point.z), size.z/2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(point.y,    size.y/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(point.y, size.y/-2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(point.x,    size.x/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(point.x, size.x/-2.0, SURFACE_POINT_CHECK_TOLERANCE));
}

bool Cube::check_point_on_edge(glm::dvec3 point) const
{
    // convert point to local coordinate system
    point = glm::inverse(_orientation_matrix) * (point - _position);

    return (solver::check_value_equal(abs(point.x), size.x/2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_equal(abs(point.y), size.y/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(     point.z,  size.z/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(  point.z,  size.z/-2.0, SURFACE_POINT_CHECK_TOLERANCE)) ||

           (solver::check_value_equal(abs(point.x), size.x/2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_equal(abs(point.z), size.z/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(     point.y,  size.y/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(  point.y,  size.y/-2.0, SURFACE_POINT_CHECK_TOLERANCE)) ||

           (solver::check_value_equal(abs(point.y), size.y/2.0, SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_equal(abs(point.z), size.z/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_less(     point.x,  size.x/2.0,  SURFACE_POINT_CHECK_TOLERANCE) &&
            solver::check_value_greater(  point.x,  size.x/-2.0, SURFACE_POINT_CHECK_TOLERANCE));
}

glm::dvec3 Cube::get_point_velocity(const glm::dvec3 &point) const
{
    return _velocity + glm::cross(_angular_velocity, point - _position);
}

glm::dvec3 Cube::get_position() const
{
    return _position;
}

glm::dvec3 Cube::get_point_r(const glm::dvec3 &point) const
{
    return glm::inverse(_orientation_matrix) * (point - _position);
}

glm::dmat3x3 Cube::get_inverse_inertia_tensor() const
{
    return _body_inertia_tensor_inv;
}

std::array<double, 13> Cube::dxdt()
{
    std::array<double, 13> output;

    // position derivative is linear velocity
    output[0] = _velocity.x;
    output[1] = _velocity.y;
    output[2] = _velocity.z;

    // orientation derivate is angular velocity
    glm::dquat tmp(0, _angular_velocity);
    glm::dquat orientation_deriv = 0.5 * (tmp * _orientation);
    output[3] = orientation_deriv.x;
    output[4] = orientation_deriv.y;
    output[5] = orientation_deriv.z;
    output[6] = orientation_deriv.w;

    // linear momentum derivative is applied force
    output[7] = _current_force.x;
    output[8] = _current_force.y;
    output[9] = _current_force.z;

    // angular momentum derivative is applied torque
    output[10] = _current_torque.x;
    output[11] = _current_torque.y;
    output[12] = _current_torque.z;

    return output;

}


std::array<double, 13> Cube::state_as_array()
{
    std::array<double, 13> output;

    output[0] = _position.x;
    output[1] = _position.y;
    output[2] = _position.z;

    output[3] = _orientation.x;
    output[4] = _orientation.y;
    output[5] = _orientation.z;
    output[6] = _orientation.w;

    output[7] = _linear_momentum.x;
    output[8] = _linear_momentum.y;
    output[9] = _linear_momentum.z;

    output[10] = _angular_momentum.x;
    output[11] = _angular_momentum.y;
    output[12] = _angular_momentum.z;

    return output;

}


void Cube::update_from_array(const std::array<double, 13> &state)
{
    _position.x = state[0];
    _position.y = state[1];
    _position.z = state[2];

    _orientation.x = state[3];
    _orientation.y = state[4];
    _orientation.z = state[5];
    _orientation.w = state[6];

    _linear_momentum.x = state[7];
    _linear_momentum.y = state[8];
    _linear_momentum.z = state[9];

    _angular_momentum.x = state[10];
    _angular_momentum.y = state[11];
    _angular_momentum.z = state[12];

    _compute_derived_variables();
}
