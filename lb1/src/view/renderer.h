#pragma once
#include <GLFW/glfw3.h>
#include "../model/scene.h"
#include "cube_mesh.h"

class Renderer
{
public:
    Renderer(GLFWwindow *window);
    ~Renderer();

    void render(const Scene &scene);
private:
    void _prepare_opengl();

    void _draw_mesh(unsigned VAO, unsigned vertices_count);

    struct shader_in {
        unsigned int model_transform;
        unsigned int camera_transform;
        unsigned int projection_transform;
    };
    
    GLFWwindow *_window;
    
    unsigned int _shader;
    shader_in _shader_data;
};
