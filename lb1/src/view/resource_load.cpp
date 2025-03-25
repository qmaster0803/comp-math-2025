#include "../glad/glad.h"
#include "resource_load.h"
#include <iostream>
#include <fstream>
#include <sstream>

unsigned int resource_load::make_module(const std::string& filepath, unsigned int module_type)
{
    std::ifstream file;
    std::stringstream bufferedLines;
    std::string line;

    // read shader from the file
    file.open(filepath);
    while (std::getline(file, line)) {
        bufferedLines << line << '\n';
    }
    file.close();
    
    std::string shaderSource = bufferedLines.str();
    const char* shaderSrc = shaderSource.c_str();

    unsigned int shaderModule = glCreateShader(module_type);
    glShaderSource(shaderModule, 1, &shaderSrc, NULL);
    glCompileShader(shaderModule);

    int success;
    glGetShaderiv(shaderModule, GL_COMPILE_STATUS, &success);
    
    if (!success) {
        char errorLog[1024];
        glGetShaderInfoLog(shaderModule, 1024, NULL, errorLog);
        std::cout << "Shader Module compilation error:\n" << errorLog << std::endl;
    }

    return shaderModule;
}

unsigned int resource_load::make_shader(const std::string& vertex_filepath, const std::string& fragment_filepath)
{
    unsigned int vert_shader = make_module(vertex_filepath,   GL_VERTEX_SHADER);
    unsigned int frag_shader = make_module(fragment_filepath, GL_FRAGMENT_SHADER);

    // attach all the modules then link the program
    unsigned int result_shader = glCreateProgram();
    glAttachShader(result_shader, vert_shader);
    glAttachShader(result_shader, frag_shader);
    glLinkProgram(result_shader);

    // check the linking worked
    int success;
    glGetProgramiv(result_shader, GL_LINK_STATUS, &success);
    if (!success) {
        char errorLog[1024];
        glGetProgramInfoLog(result_shader, 1024, NULL, errorLog);
        std::cout << "Shader linking error:\n" << errorLog << '\n';
    }

    // separate modules are now unneeded and can be freed
    glDeleteShader(vert_shader);
    glDeleteShader(frag_shader);

    return result_shader;
}
