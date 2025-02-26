#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include <vector>
#include <SFML/Graphics.hpp>
#include "Vector.hpp"

namespace Math {

    class Converter {
    private:
        static double scale; // Scale factor for conversions

    public:
        // Set the scale factor
        static inline void setScale(double newScale) {
            scale = newScale;
        }

        // Get the current scale factor
        static inline double getScale() {
            return scale;
        }

        // Convert Vector to sf::Vector2f (divide by scale for SFML)
        static inline sf::Vector2f toVector2f(const Vector& v) {
            return sf::Vector2f(v.x / scale, v.y / scale);
        }

        // Convert Vector array to sf::Vector2f array (divide by scale for SFML)
        static inline void toVector2fArray(const std::vector<Vector>& vertices,
            std::vector<sf::Vector2f>& sfVertices)
        {
            sfVertices.resize(vertices.size());
            for (size_t i = 0; i < vertices.size(); i++) {
                sfVertices[i] = sf::Vector2f(vertices[i].x / scale, vertices[i].y / scale);
            }
        }

        // Convert sf::Vector2f to Vector (multiply by scale for world units)
        static inline Vector toVector(const sf::Vector2f& v) {
            return Vector(v.x * scale, v.y * scale);
        }

        // Convert sf::Vector2i to Vector (multiply by scale for world units)
        static inline Vector toVector(const sf::Vector2i& v) {
            return Vector(v.x * scale, v.y * scale);
        }

    };
} // namespace Math

#endif // CONVERTER_HPP
