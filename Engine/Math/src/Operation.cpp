#include <cmath>

using std::sqrt;

#include "../headers/Operation.hpp"
#include "../headers/Vector.hpp"


namespace Math {
    Vector Operation::ArtimeticMean(const std::vector<Vector>& vertices) {
        double sumX = 0.0f;
        double sumY = 0.0f;
        for (int i = 0; i < vertices.size(); i++) {
            sumX += vertices[i].x;
            sumY += vertices[i].y;
        }
        return Vector(sumX / vertices.size(), sumY / vertices.size());
    }
} // namespace Math