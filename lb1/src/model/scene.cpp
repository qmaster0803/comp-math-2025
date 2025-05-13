#include <glm/ext/vector_double4.hpp>
#include <glm/fwd.hpp>
#include <random>

#include "scene.h"
#include "../compute/solver.h"
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>

Scene::Scene()
{
    _camera = new Camera();
    this->rotate_camera(0, 0); // just to update camera position

    // falling one
    _cubes.emplace_back(glm::dvec3({0.0f, 0.0f, 2.0f}),
                        glm::dvec3({1.0f, 1.0f, 1.0f}), 1.0f);

    // steady one
    _cubes.emplace_back(glm::dvec3({0.0f, 0.0f, 0.0f}),
                        glm::dvec3({0.0f, 0.5f, 0.0f}),
                        glm::dvec3({0.2f, 10.0f, 10.0f}), 10.0f);
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
    #ifdef USE_EULER
    solver::euler_solver(_cubes[0], dt);
    solver::euler_solver(_cubes[1], dt);
    #endif
    #ifdef USE_RK4
    solver::rk4_solver(_cubes[0], dt);
    solver::rk4_solver(_cubes[1], dt);
    #endif
    #ifdef USE_RK5
    solver::rk5_solver(_cubes[0], dt);
    solver::rk5_solver(_cubes[1], dt);
    #endif
    _cubes[0].set_force_and_torque(glm::dvec3({0, 0, 0}), glm::dvec3({0, 0, 0}));
    auto contacts = get_contacts();
    process_contacts(contacts);
}

void Scene::apply_action()
{
    static bool applied = false;
    // _cubes[0].set_force_and_torque(glm::dvec3({0, 0, -1}), glm::dvec3({0, 0, 0}));
    if(!applied) {
        _cubes[0].apply_impulse(glm::dvec3{0, 0, -2.5}, glm::dvec3{0, 0, 0});
        applied = true;
    }
}

void Scene::process_contacts(const std::vector<Contact> &contacts)
{
    std::array<glm::dvec3, 2> resulting_forces  = {glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 0.0)};
    std::array<glm::dvec3, 2> resulting_torques = {glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 0.0)};
    unsigned sum_elements = 0;

    for(const auto &contact : contacts) {
        // calculate relative velocity of cubes at the contact point
        const glm::dvec3 cube0_pv = _cubes[contact.body_a].get_point_velocity(contact.point);
        const glm::dvec3 cube1_pv = _cubes[contact.body_b].get_point_velocity(contact.point);
        const glm::dvec3 normal_unit = glm::normalize(contact.normal);
        const double contact_velocity = glm::dot(normal_unit, cube0_pv - cube1_pv);
        // std::cout << "contact.point: (" << contact.point.x << "; " << contact.point.y << "; " << contact.point.z << ")" << std::endl;
        // std::cout << "contact.normal: (" << contact.normal.x << "; " << contact.normal.y << "; " << contact.normal.z << ")" << std::endl;
        // std::cout << "cube0_pv: (" << cube0_pv.x << "; " << cube0_pv.y << "; " << cube0_pv.z << ")" << std::endl;
        // std::cout << "cube1_pv: (" << cube1_pv.x << "; " << cube1_pv.y << "; " << cube1_pv.z << ")" << std::endl;
        // std::cout << "Vertex to face: " << contact.vertex_to_face << std::endl;
        std::cout << "Contact velocity: " << contact_velocity << std::endl;

        if(contact_velocity < -MIN_COLLISION_SPEED) {
            // const glm::dvec3 ra = _cubes[contact.body_a].get_point_r(contact.point);
            // const glm::dvec3 rb = _cubes[contact.body_b].get_point_r(contact.point);
            const glm::dvec3 ra = (contact.point) - _cubes[contact.body_a].get_position();
            const glm::dvec3 rb = (contact.point) - _cubes[contact.body_b].get_position();

            double bouncy = 1.0;
            #ifdef ELASTIC
                #ifdef USE_RK4
                bouncy = 3.576;
                #endif
                #ifdef USE_RK5
                bouncy = 3.57529;
                #endif
            #endif
            const double num = -(1.0 + bouncy) * contact_velocity;
            const double denom = (1.0 / _cubes[contact.body_a].mass) +
                                 (1.0 / _cubes[contact.body_b].mass) +
                                 glm::dot(normal_unit, glm::cross(_cubes[contact.body_a].get_inverse_inertia_tensor() * glm::cross(ra, normal_unit), ra)) +
                                 glm::dot(normal_unit, glm::cross(_cubes[contact.body_b].get_inverse_inertia_tensor() * glm::cross(rb, normal_unit), rb));

            const double j = num / denom;
            const glm::dvec3 force = j * normal_unit;
            const glm::dvec3 torque_a = glm::cross(ra, force);
            const glm::dvec3 torque_b = glm::cross(rb, force);
            // const glm::dvec3 torque_a = {0,0,0};
            // const glm::dvec3 torque_b = {0,0,0};

            // std::cout << "Applied impulse:" << std::endl;
            // std::cout << "force: (" << force.x << "; " << force.y << "; " << force.z << ")" << std::endl;
            // std::cout << "torque_a: (" << torque_a.x << "; " << torque_a.y << "; " << torque_a.z << ")" << std::endl;
            // std::cout << "torque_b: (" << torque_b.x << "; " << torque_b.y << "; " << torque_b.z << ")" << std::endl;

            resulting_forces[contact.body_a]  += force;
            resulting_torques[contact.body_a] += torque_a;
            resulting_forces[contact.body_b]  += -force;
            resulting_torques[contact.body_b] += -torque_b;
            ++sum_elements;
        }
        std::cout << std::endl;
    }
    if(sum_elements != 0) {
        resulting_forces[0] /= sum_elements;
        resulting_forces[1] /= sum_elements;
        resulting_torques[0] /= sum_elements;
        resulting_torques[1] /= sum_elements;
        std::cout << "Applied impulse:" << std::endl;
        std::cout << "resulting_forces[0]: (" << resulting_forces[0].x << "; " << resulting_forces[0].y << "; " << resulting_forces[0].z << ")" << std::endl;
        std::cout << "resulting_forces[1]: (" << resulting_forces[1].x << "; " << resulting_forces[1].y << "; " << resulting_forces[1].z << ")" << std::endl;
        std::cout << "resulting_torques[0]: (" << resulting_torques[0].x << "; " << resulting_torques[0].y << "; " << resulting_torques[0].z << ")" << std::endl;
        std::cout << "resulting_torques[1]: (" << resulting_torques[1].x << "; " << resulting_torques[1].y << "; " << resulting_torques[1].z << ")" << std::endl;
        _cubes[0].apply_impulse(resulting_forces[0], resulting_torques[0]);
        _cubes[1].apply_impulse(resulting_forces[1], resulting_torques[1]);
    }
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
    double KE0 = _cubes[0].get_kinetic_energy();
    double KE1 = _cubes[1].get_kinetic_energy();
    std::cout << "c=" << count++ << "; found=" << curr_sep_planes.size() << "; KE0=" <<
                 KE0 << "; KE1=" << KE1 << "; KE_sum=" << KE0 + KE1 << std::endl;
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
        const glm::dvec3 normal = glm::dvec3(plane.plane.x, plane.plane.y, plane.plane.z);
        double D = plane.plane.w * -1.0;
        
        const auto &vertices_to_check = (plane.from_cube_0) ? vertices1 : vertices0;
        const auto &plane_cube = (plane.from_cube_0) ? _cubes[0] : _cubes[1];
        const auto &vertex_cube = (plane.from_cube_0) ? _cubes[1] : _cubes[0];
        const unsigned body_a = (plane.from_cube_0) ? 1 : 0;
        const unsigned body_b = (plane.from_cube_0) ? 0 : 1;
        
        for(const auto &vertex : vertices_to_check) {
            double dot_product = glm::dot(normal, vertex);
            if(solver::check_value_equal(dot_product, D, CONTACT_EPSILON) &&
               plane_cube.check_point_on_surface(vertex)) {
                // contact found

                // check normal direction
                // normal vector should be pointing towards body A
                double tmp = glm::dot(normal, vertex_cube.get_position() - vertex);
                result.emplace_back(body_a, body_b, vertex, (tmp <= 0) ? -normal : normal);
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

    for(const auto &edge0 : edges0) {
        // std::cout << "Edge 0 next" << std::endl;
        // std::cout << "--------------------------------------------" << std::endl;
        for(const auto &edge1 : edges1) {
            const glm::dvec3 n0 = (edge0.second - edge0.first);
            const glm::dvec3 n1 = (edge1.second - edge1.first);
            const glm::dvec3 ndiff = (edge1.first - edge0.first);

            const double triple_product = glm::dot(n0, glm::cross(n1, ndiff));
            if(std::abs(triple_product) <= COMPLANARITY_EPSILON)
                continue;

            const glm::dvec3 cross_product = glm::cross(n0, n1);

            const double dist = std::abs(triple_product) / std::abs(glm::length(cross_product));
            if(dist <= CONTACT_EPSILON) {
                // std::cout << "M0: (" << edge0.first.x << "; " << edge0.first.y << "; " << edge0.first.z << ")" << std::endl;
                // std::cout << "M0e: (" << edge0.second.x << "; " << edge0.second.y << "; " << edge0.second.z << ")" << std::endl;
                // std::cout << "M1: (" << edge1.first.x << "; " << edge1.first.y << "; " << edge1.first.z << ")" << std::endl;
                // std::cout << "n0: (" << n0.x << "; " << n0.y << "; " << n0.z << ")" << std::endl;
                // std::cout << "n1: (" << n1.x << "; " << n1.y << "; " << n1.z << ")" << std::endl;
                // std::cout << dist << std::endl;
                // std::cout << "dist ok" << std::endl;
                // std::cout << "Diff: (" << ndiff.x << "; " << ndiff.y << "; " << ndiff.z << ")" << std::endl;
            
                // std::cout << triple_product << std::endl;
                // find perpendicular to this edges - it's middle point will be a contact point
                // from parametric line equation: (h1/h2 are common points of perpendicular and edge0/edge1)
                // h1 = (n0.x * t + edge0.first.x; n0.y * t + edge0.first.y; n0.z * t + edge0.first.z)
                // h2 = (n1.x * s + edge1.first.x; n1.y * s + edge1.first.y; n1.z * s + edge1.first.z)
                // h1h2 = h2 - h1
                // 
                // geometrically cross_product is perpendicular to both of the edges, so it's a normal of perpendicular line
                // lambda = cross_product
                
                // let's define system of linear equations in matrix form
                //   t  s  lambda C
                // x
                // y
                // z

                glm::dmat3x4 h1h2 = {-n0.x, n1.x, -cross_product.x, edge0.first.x - edge1.first.x,
                                     -n0.y, n1.y, -cross_product.y, edge0.first.y - edge1.first.y,
                                     -n0.z, n1.z, -cross_product.z, edge0.first.z - edge1.first.z};
                                                                  
                solver::reduce_to_RREF(h1h2);

                const double t = h1h2[0][3];
                const double s = h1h2[1][3];
                
                const glm::dvec3 h1 = {n0.x * t + edge0.first.x, n0.y * t + edge0.first.y, n0.z * t + edge0.first.z};
                const glm::dvec3 h2 = {n1.x * s + edge1.first.x, n1.y * s + edge1.first.y, n1.z * s + edge1.first.z};

                // std::cout << "checks: " << _cubes[0].check_point_on_edge(h1) << "; " << _cubes[1].check_point_on_edge(h2) << std::endl;
                const bool checks_done = _cubes[0].check_point_on_edge(h1) && _cubes[1].check_point_on_edge(h2);
                if(!checks_done)
                    continue;

                const glm::dvec3 contact_point = (h1+h2) / 2.0;
                // std::cout << "Edge contact, ";
                // std::cout << "Point(a): (" << contact_point.x << "; " << contact_point.y << "; " << contact_point.z << ")" << std::endl;
                // std::cout << "Point(1): (" << h1.x << "; " << h1.y << "; " << h1.z << ")" << std::endl;
                // std::cout << "Point(2): (" << h2.x << "; " << h2.y << "; " << h2.z << ")" << std::endl;
                // std::cout << std::endl;

                // normal vector should be pointing towards body A
                glm::dvec3 normal = glm::cross(n0, n1);
                double tmp = glm::dot(normal, _cubes[0].get_position() - contact_point);
                if(tmp <= 0)
                    normal = -normal;
                result.emplace_back(0, 1, contact_point, n0, n1, normal);
            }
        }
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
