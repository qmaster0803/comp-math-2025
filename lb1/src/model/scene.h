#pragma once
#include <functional>
#include <glm/glm.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <vector>
#include <deque>
#include "camera.h"
#include "cube.h"

#define CAMERA_DIST    15.0f
#define CONTACT_EPSILON 0.05
#define COMPLANARITY_EPSILON 0.00001
#define CONTACT_BOUNCY 1.0
#define MIN_COLLISION_SPEED 0.01

struct Contact {
    unsigned body_a, body_b;
    glm::dvec3 point;
    glm::dvec3 normal; // normal of face pointing outwards (towards body A)
    glm::dvec3 edge_a, edge_b; // contacting edges
    bool vertex_to_face;  // true=vertex/face, false=edge/edge

    // vertex-face contact constructor
    Contact(unsigned body_a, unsigned body_b, glm::dvec3 point, glm::dvec3 normal) :
        body_a{body_a}, body_b{body_b}, point{point}, normal{normal}, vertex_to_face{true} {}

    // edge-edge contact constructor
    Contact(unsigned body_a, unsigned body_b, glm::dvec3 point,
            glm::dvec3 edge_a, glm::dvec3 edge_b, glm::dvec3 normal) :
        body_a{body_a}, body_b{body_b}, point{point}, edge_a{edge_a},
        edge_b{edge_b}, vertex_to_face{false}, normal{normal} {}
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
    void process_contacts(const std::vector<Contact> &contacts);

    void rotate_camera(float angle_x, float angle_y);
    void apply_debug();

private:
    Camera *_camera;
    std::vector<Cube> _cubes;

    // camera position in spherical coordinates (radians)
    float _camera_phi = glm::pi<float>() / 4.0f;
    float _camera_theta = glm::pi<float>() / 4.0f;
};
