#ifndef SHAPES_HPP
#define SHAPES_HPP

#define _USE_MATH_DEFINES
#include <cmath>

#include <SFML/Graphics.hpp>
#include "Color.hpp"

namespace Graphics {

    static constexpr float pixelPerMeter = 100.0f; // 100 pixels per meter (adjust based on zoom level)


    /** @brief Base class for all shapes.
     *
     * This serves as a base structure for all shapes such as rectangles, circles, polygons, etc.
     * It contains common properties like border thickness and color, along with virtual functions
     * for retrieving the shape and applying borders.
     */
    struct Shape {
        /** @brief Thickness of the border of the shape. */
        float borderThickness = 0;

        /** @brief Color of the border of the shape. */
        Graphics::Color borderColor = Graphics::Color::Transparent;

        /** @brief Virtual function to retrieve the shape, to be overridden by derived shapes.
         *
         * @return A reference to the shape object that should be rendered.
         */
        virtual sf::Drawable& getShape() = 0;

        /** @brief Method to set border properties.
         *
         * @param thickness Border thickness to be applied.
         * @param color Color of the border.
         */
        void addBorder(float thickness, Graphics::Color color) {
            borderThickness = thickness;
            borderColor = color;
            applyBorder(); // Apply border to the shape
        }

        /** @brief Virtual function to apply border properties to the shape.
         *
         * This should be overridden in derived classes to apply specific borders.
         */
        virtual void applyBorder() = 0;

        virtual ~Shape() = default; // Virtual destructor to ensure proper cleanup
    };

    /** @brief Rectangle shape.
     *
     * Represents a rectangle, which can have a fill color and a border. It supports rotation
     * and positioning.
     */
    struct Rectangle : public Shape {
        sf::RectangleShape shape;

        /** @brief Constructor to initialize the rectangle with properties.
         *
         * @param width The width of the rectangle in meters.
         * @param height The height of the rectangle in meters.
         * @param angle Rotation angle in radians.
         * @param position Position of the rectangle (x, y).
         * @param borderColor Color of the rectangle's border.
         * @param borderThickness Thickness of the border.
         * @param fillColor Fill color of the rectangle.
         */
        Rectangle(float width, float height, float angle, const sf::Vector2f& position,
            Graphics::Color borderColor = Graphics::Color::Transparent, float borderThickness = 0,
            Graphics::Color fillColor = Graphics::Color::Transparent)
        {
            shape.setSize(sf::Vector2f(width, height));  // Set size using width and height
            shape.setPosition(position);
            shape.setOrigin(width / 2, height / 2);  // Set origin to the center of the rectangle
            shape.setRotation(angle * 180.f / M_PI);  // Set rotation
            shape.setFillColor(fillColor.toSFML());

            // Set border properties
            this->borderThickness = borderThickness;
            this->borderColor = borderColor;

            applyBorder();  // Apply border during construction
        }

        sf::Drawable& getShape() override {
            return shape;
        }

        /** @brief Applies the border to the rectangle.
         *
         * This method overrides the base class's virtual function to apply border properties
         * (thickness and color) to the rectangle.
         */
        void applyBorder() override {
            shape.setOutlineThickness(borderThickness);
            shape.setOutlineColor(borderColor.toSFML());
        }
    };

    /** @brief Circle shape.
     *
     * Represents a circle, which can have a fill color and a border.
     */
    struct Circle : public Shape {
        sf::CircleShape shape;

        /** @brief Constructor to initialize the circle with properties.
         *
         * @param radius The radius of the circle in meters.
         * @param x X position of the circle.
         * @param y Y position of the circle.
         * @param borderColor Color of the circle's border.
         * @param borderThickness Thickness of the border.
         * @param fillColor Fill color of the circle.
         */
        Circle(float radius, float x, float y, Graphics::Color borderColor = Graphics::Color::Transparent,
            float borderThickness = 0, Graphics::Color fillColor = Graphics::Color::Transparent)
        {
            shape.setRadius(radius);
            shape.setOrigin(radius, radius); // Set the origin to the center of the circle
            shape.setPosition(x, y);  // Set position using x and y
            shape.setFillColor(fillColor.toSFML());

            // Set border properties
            this->borderThickness = borderThickness;
            this->borderColor = borderColor;

            applyBorder();  // Apply border during construction
        }

        sf::Drawable& getShape() override {
            return shape;
        }

        /** @brief Applies the border to the circle.
         *
         * This method overrides the base class's virtual function to apply border properties
         * (thickness and color) to the circle.
         */
        void applyBorder() override {
            shape.setOutlineThickness(borderThickness);
            shape.setOutlineColor(borderColor.toSFML());
        }
    };

    /** @brief Polygon shape.
     *
     * Represents a polygon with arbitrary vertices. It can have a fill color and a border.
     */
    struct Polygon : public Shape {
        sf::ConvexShape shape;

        /** @brief Constructor to initialize the polygon with vertices.
         *
         * @param points A vector of points (vertices) that define the polygon.
         * @param borderColor Color of the polygon's border.
         * @param borderThickness Thickness of the border.
         * @param fillColor Fill color of the polygon.
         */
        Polygon(const std::vector<sf::Vector2f>& points, Graphics::Color borderColor = Graphics::Color::Transparent,
            float borderThickness = 0, Graphics::Color fillColor = Graphics::Color::Transparent)
        {
            shape.setPointCount(points.size());
            for (size_t i = 0; i < points.size(); i++) {
                shape.setPoint(i, points[i]);
            }
            shape.setFillColor(fillColor.toSFML());

            // Set border properties
            this->borderThickness = borderThickness;
            this->borderColor = borderColor;

            applyBorder();  // Apply border during construction
        }

        sf::Drawable& getShape() override {
            return shape;
        }

        /** @brief Applies the border to the polygon.
         *
         * This method overrides the base class's virtual function to apply border properties
         * (thickness and color) to the polygon.
         */
        void applyBorder() override {
            shape.setOutlineThickness(borderThickness);
            shape.setOutlineColor(borderColor.toSFML());
        }
    };

    /** @brief Line shape.
     *
     * Represents a line between two points. It does not have a border, only a color.
     */
    struct Line : public Shape {
        sf::VertexArray shape;

        /** @brief Constructor to initialize the line with two points.
         *
         * @param p1 The starting point of the line.
         * @param p2 The ending point of the line.
         * @param color The color of the line.
         */
        Line(const sf::Vector2f& p1, const sf::Vector2f& p2, Graphics::Color color) {
            shape.setPrimitiveType(sf::Lines);
            shape.resize(2);
            shape[0].position = p1;
            shape[1].position = p2;
            shape[0].color = color.toSFML();
            shape[1].color = color.toSFML();
            shape[0].color = color.toSFML();
            shape[1].color = color.toSFML();
            applyBorder();  // Apply border during construction
        }

        sf::Drawable& getShape() override {
            return shape;
        }

        /** @brief No border for lines. */
        void applyBorder() override {}
    };

    /** @brief Text shape.
     *
     * Represents a text object that can be rendered, with support for dynamic scaling, coloring,
     * and positioning.
     */
    struct Text : public Shape {
        sf::Text text;
        static sf::Font font; // Shared font for all Text objects
        static bool isFontLoaded; // Flag to check if the font is loaded

        /** @brief Static font loader.
         *
         * Loads the font from the specified file path if not already loaded.
         *
         * @param fontPath The file path to the font.
         * @throw std::runtime_error If the font fails to load.
         */
        static void loadFont(const std::string& fontPath) {
            if (!isFontLoaded) {
                if (!font.loadFromFile(fontPath)) {
                    throw std::runtime_error("Failed to load: " + fontPath);
                }
                isFontLoaded = true;
            }
        }

        /** @brief Constructor to initialize the text.
         *
         * @param content The content of the text.
         * @param position The position of the text.
         * @param color The color of the text.
         * @param textSizeInMeters The size of the text in meters (scaled to pixels).
         * @param setOriginToCenter Whether to set the origin to the center of the text.
         * @throw std::runtime_error If the font is not loaded.
         */
        Text(const std::string& content, const sf::Vector2f& position, Graphics::Color color = Graphics::Color::White,
            float textSizeInMeters = 0.2f, bool setOriginToCenter = true)
        {
            if (!isFontLoaded) {
                throw std::runtime_error("Font not loaded. Call loadFont() before creating Text objects");
            }
            text.setFont(font);
            text.setString(content);
            text.setFillColor(color.toSFML());
            text.setCharacterSize(static_cast<unsigned int>(textSizeInMeters * pixelPerMeter));
            text.setScale(1.0f / pixelPerMeter, -1.0f / pixelPerMeter); // Inverse scaling for consistency

            if (setOriginToCenter) {
                // Set the origin to the center of the text
                sf::FloatRect textBounds = text.getLocalBounds();
                text.setOrigin(
                    textBounds.left + textBounds.width / 2.0f,
                    textBounds.top + textBounds.height / 2.0f);
            }

            text.setPosition(position);
        }

        /** @brief Method to scale the text size.
         *
         * @param scale Scale factor.
         */
        void setScale(float scale) {
            text.setScale(scale / pixelPerMeter, -scale / pixelPerMeter);
        }

        sf::Drawable& getShape() override {
            return text;
        }

        /** @brief Static method to calculate text size in pixel-meters.
         *
         * @param content The content of the text.
         * @param textSizeInMeters The size of the text in meters.
         * @return The size of the text in pixel-meters.
         * @throw std::runtime_error If the font is not loaded.
         */
        static sf::Vector2f getTextSize(const std::string& content, float textSizeInMeters = 0.2f) {
            if (!isFontLoaded) {
                throw std::runtime_error("Font not loaded. Call loadFont() before using getTextSize()");
            }

            sf::Text textObj;
            textObj.setFont(font);
            textObj.setString(content);

            // Set the character size in pixels, based on the text size in meters
            textObj.setCharacterSize(static_cast<unsigned int>(textSizeInMeters * pixelPerMeter));

            // Get the bounds of the text in pixels
            sf::FloatRect textBounds = textObj.getLocalBounds();

            // Return the size in "pixel-meters" by dividing by the pixelPerMeter
            return sf::Vector2f(textBounds.width / pixelPerMeter, textBounds.height / pixelPerMeter);
        }

        /** @brief No border for text. */
        void applyBorder() override {}
    };

    // Define the static font outside the class
    sf::Font Text::font;
    bool Text::isFontLoaded = false;

} // namespace Graphics

#endif // SHAPES_HPP
