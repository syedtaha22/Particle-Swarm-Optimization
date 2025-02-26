/*

    Contains all the math functions and classes used in the physics engine.

*/

#ifndef OPERATION_HPP
#define OPERATION_HPP

#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include "Vector.hpp"

namespace Math {


    class Operation {

    public:

        // Dot Product
        static inline double DotProduct(const Vector& v1, const Vector& v2) {
            return v1.x * v2.x + v1.y * v2.y;
        }

        // Cross Product
        static inline double CrossProduct(const Vector& v1, const Vector& v2) {
            return v1.x * v2.y - v1.y * v2.x;
        }

        // Lenght of a vector
        static inline double Length(const Vector& v) {
            return sqrt(v.x * v.x + v.y * v.y);
        }

        // Normalize a vector
        static inline Vector Normalize(const Vector& v) {
            double len = Length(v);
            return Vector(v.x / len, v.y / len);
        }

        // Distance between two points
        static inline double Distance(const Vector& v1, const Vector& v2) {
            return Length(v1 - v2);
        }

        // Squared Distance between two points
        static inline double SquaredDistance(const Vector& v1, const Vector& v2) {
            return (v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y);
        }

        // Clamp
        template <typename T>
        static T Clamp(T value, T min, T max) {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        static Vector ArtimeticMean(const std::vector<Vector>& vertices);

        // Edge between two points
        static inline Vector EdgeBetween(const Vector& p1, const Vector& p2) {
            /*
                Edge from point 1 to point 2:
                Edge = Point 2 - Point 1
            */
            return p2 - p1;
        }

        // Round a number
        static inline double Round(double number, size_t decimalPlaces) {
            double factor = static_cast<double>(std::pow(10, decimalPlaces));
            return std::round(std::trunc(number * factor)) / factor;
        }

        // Round a number and return as a string
        static inline std::string RoundToString(double number, size_t decimalPlaces) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(decimalPlaces) << number;
            return oss.str();
        }


        static inline bool NearlyEqual(double a, double b, double epsilon) {
            return std::abs(a - b) < epsilon;
        }

        // Check if two vectors are nearly equal
        static inline bool NearlyEqual(const Vector& v1, const Vector& v2, double epsilon) {
            return SquaredDistance(v1, v2) < epsilon * epsilon;
        }

        // Get the perpendicular vector
        static inline Vector Perpendicular(const Vector& v) {
            return Vector(-v.y, v.x);
        }

    };
} // namespace Math

#endif // OPERATION_HPP