#include "../glad/glad.h"

#include "app.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <functional>

App::App(std::size_t window_width, std::size_t window_height) :
    _win_width{window_width}, _win_height{window_height}
{
    _setup_glfw();
    _scene = new Scene();
    _renderer = new Renderer(_window);
}

App::~App()
{
    delete _scene;
    delete _renderer;
    glfwTerminate();
}

void App::_setup_glfw()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    _window = glfwCreateWindow(_win_width, _win_height, "Rubik's cube solver", NULL, NULL);
    glfwMakeContextCurrent(_window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Couldn't load opengl" << std::endl;
        glfwTerminate();
    }
}

void App::_handle_input()
{
    if (glfwGetKey(_window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        _scene->apply_debug();
    }

    
    if (glfwGetWindowAttrib(_window, GLFW_HOVERED)) {
        static double prev_mouse_x = 0, prev_mouse_y = 0;
        double mouse_x, mouse_y;
        glfwGetCursorPos(_window, &mouse_x, &mouse_y);

        if(glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT)) {
            _scene->rotate_camera(prev_mouse_x - mouse_x, prev_mouse_y - mouse_y);
        }
        prev_mouse_x = mouse_x;
        prev_mouse_y = mouse_y;
    }
}

void App::run()
{
    std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
    while(!glfwWindowShouldClose(_window)) {
        glfwPollEvents();
        _handle_input();
        
        std::chrono::steady_clock::time_point timestamp_new = std::chrono::steady_clock::now();
        float seconds_on_frame = std::chrono::duration_cast<std::chrono::microseconds>(timestamp_new - timestamp).count() / 1000000.0f;
        _scene->update(seconds_on_frame);
        _renderer->render(*_scene);
        timestamp = timestamp_new;
    }
}
