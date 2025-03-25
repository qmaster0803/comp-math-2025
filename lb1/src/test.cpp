#include <iostream>
#include <glm/glm.hpp>
#include <optional>

using namespace glm;
using namespace std;

void ReduceToRREF(glm::dmat3& matrix) {
    int lead = 0;
    int rowCount = matrix.length();
    int colCount = matrix[0].length();

    for (int r = 0; r < rowCount; r++) {
        if (lead >= colCount) break;

        int i = r;
        while (matrix[i][lead] == 0) {
            i++;
            if (i == rowCount) {
                i = r;
                lead++;
                if (lead == colCount) return;
            }
        }

        // Меняем строки местами, если нужно
        if (i != r) {
            for (int k = 0; k < colCount; k++) {
                std::swap(matrix[i][k], matrix[r][k]);
            }
        }

        // Нормализуем ведущую строку
        float val = matrix[r][lead];
        for (int k = 0; k < colCount; k++) {
            matrix[r][k] /= val;
        }

        // Обнуляем элементы в ведущем столбце
        for (int i = 0; i < rowCount; i++) {
            if (i != r) {
                float factor = matrix[i][lead];
                for (int k = 0; k < colCount; k++) {
                    matrix[i][k] -= factor * matrix[r][k];
                }
            }
        }

        lead++;
    }
}

std::optional<glm::vec3> intersectSegments(
    const glm::vec3& A, const glm::vec3& B,
    const glm::vec3& C, const glm::vec3& D,
    float epsilon = 1e-6f) {
    
    // Directional vectors
    const glm::dvec3 dir1 = B - A;
    const glm::dvec3 dir2 = D - C;

    // Now this segments can be represented in this way:
    // first(t) = dir1*t + A
    // second(s) = dir2*s + C
    // Where t,s = 0 means "start point" and t,s = 1 is located at the end of the segment

    // Vector equation for intersection point:
    // dir1 * t + A = dir2 * s + C, while 0 <= t,s <= 1
    
    // In linear non-vector form
    // dir1.x * t + A.x - dir2.x * s - C.x = 0
    // dir1.y * t + A.y - dir2.y * s - C.y = 0
    // dir1.z * t + A.z - dir2.z * s - C.z = 0

    // In this specific task we also need to implement epsilong logic
    // So our linear equations become linear inequations
    // abs(dir1.x * t + A.x - dir2.x * s - C.x) <= epsilon
    // abs(dir1.y * t + A.y - dir2.y * s - C.y) <= epsilon
    // abs(dir1.z * t + A.z - dir2.z * s - C.z) <= epsilon

    // Let's use first two equations to find t and s
    

    // This system of linear equations can be solved using matrix
    glm::dmat3 eq_mat = {dir1.x, dir2.x, (C.x - A.x),
                         dir1.y, dir2.y, (C.y - A.y),
                         dir1.z, dir2.z, (C.z - A.z)};
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << eq_mat[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    ReduceToRREF(eq_mat);
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << eq_mat[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    return nullopt;
}

int main() {
    // Пример 1: Прямые пересекаются в точке (7, 11, 15)
    auto intersection = intersectSegments(vec3(1, 2, 3), vec3(2, 3, 4), vec3(2, 1, 0), vec3(1, 2, 10));

    if (intersection) {
        cout << "Точка пересечения: (" 
             << intersection->x << ", " 
             << intersection->y << ", " 
             << intersection->z << ")\n";
    } else {
        cout << "Прямые не пересекаются.\n";
    }

    // Пример 2: Скрещивающиеся прямые
    intersection = intersectSegments(vec3(1, 2, 3), vec3(4, 6, 8), vec3(2, 1, 0), vec3(2, 4, 6));

    if (intersection) {
        cout << "Точка пересечения: (" 
             << intersection->x << ", " 
             << intersection->y << ", " 
             << intersection->z << ")\n";
    } else {
        cout << "Прямые не пересекаются.\n";
    }

    return 0;
}
