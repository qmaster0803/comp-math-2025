#include <iostream>
#include <glm/glm.hpp>
#include <optional>

using namespace glm;
using namespace std;

void ReduceToRREF(glm::dmat3x4& matrix) {
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

#include <iomanip>
std::optional<glm::vec3> intersectSegments(
    const glm::vec3& A, const glm::vec3& B,
    const glm::vec3& C, const glm::vec3& D,
    float epsilon = 1e-6f) {

    auto edge0 = std::make_pair(A, B);
    auto edge1 = std::make_pair(C, D);
    const glm::dvec3 n0 = (edge0.second - edge0.first);
    const glm::dvec3 n1 = (edge1.second - edge1.first);
    const glm::dvec3 ndiff = (edge1.first - edge0.first);

    const double triple_product = glm::dot(n0, glm::cross(n1, ndiff));
    if(std::abs(triple_product) <= 0.000001)
        return nullopt;

    const glm::dvec3 cross_product = glm::cross(n0, n1);

    const double dist = std::abs(triple_product) / std::abs(cross_product.length());
    if(dist <= 0.1) {
        std::cout << "M0: (" << edge0.first.x << "; " << edge0.first.y << "; " << edge0.first.z << ")" << std::endl;
        std::cout << "M0e: (" << edge0.second.x << "; " << edge0.second.y << "; " << edge0.second.z << ")" << std::endl;
        std::cout << "M1: (" << edge1.first.x << "; " << edge1.first.y << "; " << edge1.first.z << ")" << std::endl;
        std::cout << "n0: (" << n0.x << "; " << n0.y << "; " << n0.z << ")" << std::endl;
        std::cout << "n1: (" << n1.x << "; " << n1.y << "; " << n1.z << ")" << std::endl;
        std::cout << dist << std::endl;
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
                                                          
        // solver::reduce_to_RREF(h1h2);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                std::cout << std::setw(5) << h1h2[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        ReduceToRREF(h1h2);

        const double t = h1h2[0][3];
        const double s = h1h2[1][3];
        std::cout << "t=" << t << "; s=" << s << std::endl;
        
        const glm::dvec3 h1 = {n0.x * t + edge0.first.x, n0.y * t + edge0.first.y, n0.z * t + edge0.first.z};
        const glm::dvec3 h2 = {n1.x * s + edge1.first.x, n1.y * s + edge1.first.y, n1.z * s + edge1.first.z};
        std::cout << "Point h1: (" << h1.x << "; " << h1.y << "; " << h1.z << ")" << std::endl;
        std::cout << "Point h2: (" << h2.x << "; " << h2.y << "; " << h2.z << ")" << std::endl;

        const glm::dvec3 contact_point = (h1+h2) / 2.0;
        std::cout << "Edge contact, ";
        std::cout << "Point: (" << contact_point.x << "; " << contact_point.y << "; " << contact_point.z << ")" << std::endl;
    }
    return nullopt;
}

int main() {
    auto intersection = intersectSegments(vec3(1, 2, 3), vec3(2, 3, 4), vec3(2, 1, 0), vec3(1, 2, 10));

    // intersection = intersectSegments(vec3(1, 2, 3), vec3(4, 6, 8), vec3(2, 1, 0), vec3(2, 4, 6));

    intersection = intersectSegments(vec3(-0.09, -0.5, 0.5), vec3(-0.09, -0.5, -0.5), vec3(-0.1, -0.25, -10), vec3(-0.1, 0.75, -10));

    return 0;
}
