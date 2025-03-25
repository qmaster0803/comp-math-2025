#include <glm/ext/vector_double4.hpp>
#include <random>

#include "scene.h"
#include "../compute/solver.h"
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>

Scene::Scene()
{
    _camera = new Camera();
    this->rotate_camera(0, 0); // just to update camera position

    _cubes.emplace_back(glm::dvec3({-0.79f, 0.0f, 0.0f}), glm::dvec3({1.0f, 1.0f, 1.0f}), 100.0f);
    _cubes.emplace_back(glm::dvec3({0.0f, 0.0f, 0.0f}), glm::dvec3({0.2f, 0.5f, 20.0f}), 100.0f);
}

Scene::~Scene()
{
    delete _camera;
}

void Scene::rotate_camera(float angle_x, float angle_y)
{
    _camera_phi += glm::radians(angle_x);
    _camera_theta += glm::radians(angle_y);
    
    if(_camera_theta > glm::pi<float>())
        _camera_theta = glm::pi<float>();
    if(_camera_theta < 0.0f)
        _camera_theta = 0.01f;

    if(_camera_phi > (glm::pi<float>() * 2.0))
        _camera_phi -= glm::pi<float>() * 2.0;
    if(_camera_phi < (glm::pi<float>() * -2.0))
        _camera_phi += glm::pi<float>() * 2.0;

    glm::vec3 camera_pos = {CAMERA_DIST * glm::cos(_camera_phi) * glm::sin(_camera_theta),
                            CAMERA_DIST * glm::sin(_camera_phi) * glm::sin(_camera_theta),
                            CAMERA_DIST * glm::cos(_camera_theta)};

    _camera->move(camera_pos);
}

void Scene::update(float dt)
{    
    solver::euler_solver(_cubes[0], dt);
    _cubes[0].set_force_and_torque(glm::dvec3({0, 0, 0}), glm::dvec3({0, 0, 0}));
    get_contacts();
}

void Scene::apply_debug()
{
    _cubes[0].set_force_and_torque(glm::dvec3({10, 0, 0}), glm::dvec3({0, 0, 0}));
}

std::vector<Contact> Scene::get_contacts() const
{
    std::vector<Contact> result;
    
    // check for vertex to face contacts first
    // Step 1. Search for separating plane
    auto faces0 = _cubes[0].get_faces();
    auto faces1 = _cubes[1].get_faces();

    auto vertices0 = _cubes[0].get_vertices();
    auto vertices1 = _cubes[1].get_vertices();

    struct sep_plane {
        glm::dvec4 plane;
        bool from_cube_0;
        sep_plane(glm::dvec4 plane, bool from_cube_0) :
            plane{plane}, from_cube_0{from_cube_0} {}
    };
    
    static std::vector<sep_plane> prev_sep_planes;
    std::vector<sep_plane> curr_sep_planes;
    
    for(const auto &face : faces0) {
        glm::dvec3 normal = glm::dvec3(face.x, face.y, face.z);
        double D = face.w * -1.0;
        bool face_ok = true;
        for(const auto &vertex : vertices1) {
            if(!solver::check_value_greater(glm::dot(normal, vertex), D, CONTACT_EPSILON)) {
                face_ok = false;
                break;
            }
        }

        if(face_ok) {
            curr_sep_planes.emplace_back(face, true);
            break;
        }
    }

    // search for another vector of faces
    for(const auto &face : faces1) {
        glm::dvec3 normal = glm::dvec3(face.x, face.y, face.z);
        double D = face.w * -1.0;
        bool face_ok = true;
        for(const auto &vertex : vertices0) {
            if(!solver::check_value_greater(glm::dot(normal, vertex), D, CONTACT_EPSILON)) {
                face_ok = false;
                break;
            }
        }

        if(face_ok) {
            curr_sep_planes.emplace_back(face, false);
            break;
        }
    }

    static std::size_t count = 0;
    std::cout << "c=" << count++ << "; found=" << curr_sep_planes.size() << std::endl;
     // << "; plane=(" <<
     //             separating_face.x << ", " << separating_face.y << ", " <<
     //             separating_face.z << ", " << separating_face.w << "); " << 
     //             "cube0=" << separating_face_from_cube0 << std::endl;
     
    // if separation plane(s) found there's nothing more to do
    if(!curr_sep_planes.empty()) {
        prev_sep_planes = curr_sep_planes;
        return result;
    }


    // Step 2. Check last separating plane for vertex to face contacts
    // search for vertex to plane contacts
    for(const auto &plane : prev_sep_planes) {
        glm::dvec3 normal = glm::dvec3(plane.plane.x, plane.plane.y, plane.plane.z);
        double D = plane.plane.w * -1.0;
        
        const auto &vertices_to_check = (plane.from_cube_0) ? vertices1 : vertices0;
        const auto &plane_cube = (plane.from_cube_0) ? _cubes[0] : _cubes[1];
        
        for(const auto &vertex : vertices_to_check) {
            double dot_product = glm::dot(normal, vertex);
            if(solver::check_value_equal(dot_product, D, CONTACT_EPSILON) &&
               plane_cube.check_point_on_surface(vertex)) {
                // contact found
                result.emplace_back(vertex, normal);
            }
            else if(solver::check_value_less(dot_product, D, CONTACT_EPSILON)) {
                std::cerr << "Penetration detected!" << std::endl;
                throw;
            }
        }
    }
    
    // Step 3. Check for edge-edge contacts
    // 0---1  4---5
    // | U |  | D |
    // 2---3  6---7
    std::array<std::pair<glm::dvec3, glm::dvec3>, 12> edges0 = {
        std::make_pair(vertices0[0], vertices0[1]),
        std::make_pair(vertices0[1], vertices0[3]),
        std::make_pair(vertices0[3], vertices0[2]),
        std::make_pair(vertices0[2], vertices0[0]),

        std::make_pair(vertices0[0], vertices0[4]),
        std::make_pair(vertices0[1], vertices0[5]),
        std::make_pair(vertices0[2], vertices0[6]),
        std::make_pair(vertices0[3], vertices0[7]),

        std::make_pair(vertices0[4], vertices0[5]),
        std::make_pair(vertices0[5], vertices0[7]),
        std::make_pair(vertices0[7], vertices0[6]),
        std::make_pair(vertices0[6], vertices0[4])
    };
    std::array<std::pair<glm::dvec3, glm::dvec3>, 12> edges1 = {
        std::make_pair(vertices1[0], vertices1[1]),
        std::make_pair(vertices1[1], vertices1[3]),
        std::make_pair(vertices1[3], vertices1[2]),
        std::make_pair(vertices1[2], vertices1[0]),

        std::make_pair(vertices1[0], vertices1[4]),
        std::make_pair(vertices1[1], vertices1[5]),
        std::make_pair(vertices1[2], vertices1[6]),
        std::make_pair(vertices1[3], vertices1[7]),

        std::make_pair(vertices1[4], vertices1[5]),
        std::make_pair(vertices1[5], vertices1[7]),
        std::make_pair(vertices1[7], vertices1[6]),
        std::make_pair(vertices1[6], vertices1[4])
    };

    for(const auto &edge : edges0) {
        
    }

    
    std::cout << "Total contacts: " << result.size() << std::endl;
    return result;
}

glm::mat4 Scene::get_camera_transform() const
{
    return _camera->get_transform();
}

std::vector<glm::mat4> Scene::get_cubes_transform() const
{
    std::vector<glm::mat4> result;
    for(const auto &i : _cubes) {
        result.push_back(i.get_transform());
    }

    return result;
}

std::vector<unsigned> Scene::get_cube_meshes() const
{
    std::vector<unsigned> result;
    for(const auto &i : _cubes) {
        result.push_back(i.get_cube_mesh());
    }

    return result;
}
