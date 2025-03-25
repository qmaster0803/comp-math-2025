#include "../glad/glad.h"
#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "renderer.h"
#include "resource_load.h"

Renderer::Renderer(GLFWwindow *window) : _window{window}
{
    _prepare_opengl();
}

Renderer::~Renderer()
{
    glDeleteProgram(_shader);
}

void Renderer::_prepare_opengl()
{
    // clear the window with color
    glClearColor(0.25f, 0.5f, 0.75f, 1.0f);
    
    // set the rendering region to the actual screen size
    int w,h;
    glfwGetFramebufferSize(_window, &w, &h);
    // (left, top, width, height)
    glViewport(0,0,w,h);

    // enable depth test and face culling
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glEnable(GL_MULTISAMPLE);

    // load shader
    _shader = resource_load::make_shader("../src/shaders/vertex.txt", "../src/shaders/fragment.txt");
    glUseProgram(_shader);
    _shader_data.camera_transform     = glGetUniformLocation(_shader, "camera_transform");
    _shader_data.model_transform      = glGetUniformLocation(_shader, "model_transform");
    _shader_data.projection_transform = glGetUniformLocation(_shader, "projection_transform");

    // create projection transform & load it
    glm::mat4 projection = glm::perspective(45.0f, (float)w / (float)h, 0.01f, 100.0f);
    glUniformMatrix4fv(_shader_data.projection_transform, 1, GL_FALSE, glm::value_ptr(projection));
}

void Renderer::render(const Scene &scene)
{
    // clear buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // load data to the shader
    glUseProgram(_shader);
    glUniformMatrix4fv(_shader_data.camera_transform, 1, GL_FALSE, glm::value_ptr(scene.get_camera_transform()));

    auto cube_pos = scene.get_cubes_transform();
    auto cube_meshes = scene.get_cube_meshes();
    for(std::size_t i = 0; i < cube_pos.size(); ++i) {
        glUniformMatrix4fv(_shader_data.model_transform, 1, GL_FALSE, glm::value_ptr(cube_pos[i]));
        _draw_mesh(cube_meshes[i], 36);
    }

    // swap buffers (actually display the frame)
    glfwSwapBuffers(_window);
}

void Renderer::_draw_mesh(unsigned VAO, unsigned vertices_count)
{
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, vertices_count);
    // glBindVertexArray(0);
}
