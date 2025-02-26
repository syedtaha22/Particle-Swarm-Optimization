#ifndef COLOR_HPP
#define COLOR_HPP

#include <SFML/Graphics/Color.hpp>
#include <stdexcept>
#include <ostream>

namespace Graphics {

    /**
     * @brief A custom Color struct for managing RGBA color values.
     * This struct provides constructors, predefined colors, and utilities
     * for color manipulation and conversion to SFML's `sf::Color`.
     */
    struct Color {
        unsigned char r, g, b, a; // Red, Green, Blue, and Alpha (transparency) components.

        /**
         * @brief Default Constructor.
         * Initializes the color to black with full opacity.
         */
        Color()
            : r(0), g(0), b(0), a(255) {
        }

        /**
         * @brief Constructor for specifying RGBA values.
         * @param red   Red component (0-255).
         * @param green Green component (0-255).
         * @param blue  Blue component (0-255).
         * @param alpha Alpha component (0-255), default is 255 (fully opaque).
         */
        Color(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha = 255)
            : r(red), g(green), b(blue), a(alpha) {
        }

        /**
         * @brief Constructor for initializing color from a hexadecimal string.
         * Supports both "#RRGGBB" and "#RRGGBBAA" formats.
         * @param hex A string representing the color in hexadecimal format.
         * @throws std::invalid_argument if the format is invalid.
         */
        Color(const std::string& hex) {
            if (hex[0] == '#') {
                if (hex.length() == 7) {
                    // Format: #RRGGBB
                    r = std::stoi(hex.substr(1, 2), nullptr, 16);
                    g = std::stoi(hex.substr(3, 2), nullptr, 16);
                    b = std::stoi(hex.substr(5, 2), nullptr, 16);
                    a = 255; // Default to fully opaque.
                }
                else if (hex.length() == 9) {
                    // Format: #RRGGBBAA
                    r = std::stoi(hex.substr(1, 2), nullptr, 16);
                    g = std::stoi(hex.substr(3, 2), nullptr, 16);
                    b = std::stoi(hex.substr(5, 2), nullptr, 16);
                    a = std::stoi(hex.substr(7, 2), nullptr, 16);
                }
                else {
                    throw std::invalid_argument("Invalid hex color format");
                }
            }
            else {
                throw std::invalid_argument("Hex color must start with '#' character");
            }
        }

        /**
         * @brief Returns a new Color object with a modified alpha value.
         * @param alpha New alpha value (0-255).
         * @return A new Color object with the same RGB values but updated alpha.
         */
        Color withAlha(unsigned char alpha) const {
            return Color(r, g, b, alpha);
        }

        // Predefined Colors
        static const Color Red;        ///< Predefined color: Red.
        static const Color Green;      ///< Predefined color: Green.
        static const Color Blue;       ///< Predefined color: Blue.
        static const Color White;      ///< Predefined color: White.
        static const Color Black;      ///< Predefined color: Black.
        static const Color Yellow;     ///< Predefined color: Yellow.
        static const Color Orange;     ///< Predefined color: Orange.
        static const Color Transparent;///< Predefined color: Fully transparent black.

        // Dark Colors
        static const Color DarkRed;    ///< Predefined color: Dark Red.
        static const Color DarkGreen;  ///< Predefined color: Dark Green.
        static const Color DarkBlue;   ///< Predefined color: Dark Blue.

        /**
         * @brief Converts the custom Color object to an SFML `sf::Color` object.
         * @return A corresponding `sf::Color` object.
         */
        sf::Color toSFML() const {
            return sf::Color(r, g, b, a);
        }
    };

    // Definitions of predefined colors.
    const Color Color::Red = Color(255, 0, 0);
    const Color Color::Green = Color(0, 255, 0);
    const Color Color::Blue = Color(0, 0, 255);
    const Color Color::White = Color(255, 255, 255);
    const Color Color::Black = Color(0, 0, 0);
    const Color Color::Yellow = Color(255, 255, 0);
    const Color Color::Orange = Color(255, 165, 0);
    const Color Color::Transparent = Color(0, 0, 0, 0);

    // Definitions of dark colors.
    const Color Color::DarkRed = Color(139, 0, 0);
    const Color Color::DarkGreen = Color(0, 100, 0);
    const Color Color::DarkBlue = Color(0, 0, 139);

}

#endif // COLOR_HPP
