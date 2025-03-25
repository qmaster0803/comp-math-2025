#pragma once
#include <functional>
#include <glm/glm.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <vector>
#include <deque>
#include "camera.h"
#include "cube.h"

#define CAMERA_DIST    15.0f
#define CONTACT_EPSILON 0.01

struct Contact {
    glm::dvec3 point;
    glm::dvec3 normal; // normal of face pointing outwards
    glm::dvec3 edge_a, edge_b; // contacting edges
    bool vertex_to_face;  // true=vertex/face, false=edge/edge

    // vertex-face contact constructor
    Contact(glm::dvec3 point, glm::dvec3 normal) :
        point{point}, normal{normal}, vertex_to_face{true} {}

    // edge-edge contact constructor
    Contact(glm::dvec3 point, glm::dvec3 edge_a, glm::dvec3 edge_b) :
        point{point}, edge_a{edge_a}, edge_b{edge_b}, vertex_to_face{false} {}
};

class Scene
{
public:
    Scene();
    ~Scene();

    void update(float dt);
    
    glm::mat4 get_camera_transform() const;
    std::vector<glm::mat4> get_cubes_transform() const;
    std::vector<unsigned>  get_cube_meshes() const;
    std::vector<Contact>   get_contacts() const;

    void rotate_camera(float angle_x, float angle_y);
    void apply_debug();

private:
    Camera *_camera;
    std::vector<Cube> _cubes;

    // camera position in spherical coordinates (radians)
    float _camera_phi = glm::pi<float>() / 4.0f;
    float _camera_theta = glm::pi<float>() / 4.0f;
};
