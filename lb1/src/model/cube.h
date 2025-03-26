#pragma once
#include <glm/glm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <array>
#include "../view/cube_mesh.h"

#define SURFACE_POINT_CHECK_TOLERANCE 0.05

class Cube
{
public:
    const double mass;
    const glm::dvec3 size;
    Cube(glm::dvec3 initial_position, glm::dvec3 size, double mass);

    glm::mat4 get_transform() const;

    void set_force_and_torque(glm::dvec3 force, glm::dvec3 torque);
    void apply_impulse(const glm::dvec3 &linear, const glm::dvec3 &angular);
    std::array<double, 13> dxdt();
    std::array<double, 13> state_as_array();
    void update_from_array(const std::array<double, 13> &state);

    unsigned get_cube_mesh() const;

    // U, L, F, R, B, D
    std::array<glm::dvec4, 6> get_faces() const;

    // 0---1  4---5
    // | U |  | D |
    // 2---3  6---7
    std::array<glm::dvec3, 8> get_vertices() const;

    bool check_point_on_surface(glm::dvec3 point) const;
    bool check_point_on_edge(glm::dvec3 point) const;

    glm::dvec3 get_point_velocity(const glm::dvec3 &point) const;
    glm::dvec3 get_position() const;
    glm::dmat3x3 get_inverse_inertia_tensor() const;
    glm::dvec3 get_point_r(const glm::dvec3 &point) const; // radius-vector    
private:    
    // State variables
    glm::dvec3 _position;
    glm::dquat _orientation = glm::dquat(1, 0, 0, 0);
    
    glm::dvec3 _linear_momentum;
    glm::dvec3 _angular_momentum;
    
    // Constants
    glm::dmat3x3 _body_inertia_tensor;
    glm::dmat3x3 _body_inertia_tensor_inv;

    // Derived (computed) variables
    glm::dmat3x3 _orientation_matrix;
    glm::dvec3   _velocity;
    glm::dvec3   _angular_velocity;
    glm::dvec3   _current_force;
    glm::dvec3   _current_torque;

    CubeMesh *_cube_mesh;

    void _compute_derived_variables();
};
