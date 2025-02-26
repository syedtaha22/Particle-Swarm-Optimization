#include <string>


using std::string;
using std::to_string;

#include "../headers/Vector.hpp"
#include "../headers/Transformation.hpp"

namespace Math {
    // Define the zero vector
    Vector Vector::Zero = Vector(0, 0);

    Vector::Vector(double x, double y) : x(x), y(y) {}

    // Addition
    Vector Vector::operator+(const Vector& other) const { return Vector(x + other.x, y + other.y); }

    // Subtraction
    Vector Vector::operator-(const Vector& other) const { return Vector(x - other.x, y - other.y); }

    // Multiplication
    Vector Vector::operator*(double scalar) const { return Vector(x * scalar, y * scalar); }

    // Division
    Vector Vector::operator/(double scalar) const { return Vector(x / scalar, y / scalar); }

    // In-place addition
    Vector& Vector::operator+=(const Vector& other) {
        x += other.x; y += other.y;
        return *this;
    }

    // In-place subtraction
    Vector& Vector::operator-=(const Vector& other) {
        x -= other.x; y -= other.y;
        return *this;
    }

    // In-place multiplication
    Vector& Vector::operator*=(double scalar) {
        x *= scalar; y *= scalar;
        return *this;
    }

    // In-place division
    Vector& Vector::operator/=(double scalar) {
        x /= scalar; y /= scalar;
        return *this;
    }


    Vector Vector::operator-() const { return Vector(-x, -y); }

    Vector operator*(double scalar, const Vector& vector) { return Vector(vector.x * scalar, vector.y * scalar); }

    bool Vector::operator==(const Vector& other) const { return x == other.x && y == other.y; }

    std::string Vector::toString() const { return "(" + std::to_string(x) + ", " + std::to_string(y) + ")"; }

    Vector Vector::Transform(const Vector& vector, Transformation& transform) {
        return Vector(
            vector.x * transform.cosine - vector.y * transform.sine + transform.PositionX,
            vector.x * transform.sine + vector.y * transform.cosine + transform.PositionY
        );
    }

    size_t Vector::hash() const {
        return std::hash<double>()(x) ^ std::hash<double>()(y);
    }

} // namespace Math