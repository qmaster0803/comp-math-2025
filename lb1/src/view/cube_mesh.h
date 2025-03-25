#pragma once
#include <glm/glm.hpp>

class CubeMesh {

public:
    CubeMesh(glm::vec3 size = {1.0f, 1.0f, 1.0f});
    CubeMesh(CubeMesh &&another);

    unsigned get_vao() const;
private:
    unsigned int _VAO, _VBO;
    void _make_mesh(float l, float w, float h);
};
