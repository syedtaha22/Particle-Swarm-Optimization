#ifndef WINDOW_HPP
#define WINDOW_HPP

#include <iostream>
#include <SFML/Graphics.hpp>
#include "Shapes.hpp"
#include "Color.hpp"

namespace Graphics {

    /**
     * @enum TimeUnit
     * @brief Enumeration for different time units (Milliseconds, Seconds)
     */
    enum class TimeUnit { Milliseconds, Seconds };

    /**
     * @class Window
     * @brief Class that represents a window with graphics rendering capabilities, including zoom, camera, and event handling.
     */
    class Window {
    private:
        sf::RenderWindow window;       ///< The SFML render window object
        sf::View camera;               ///< Camera view for the window
        sf::Clock clock;               ///< Clock to measure elapsed time
        Graphics::Color bgColor;       ///< Background color of the window
        float zoomLevel;               ///< Current zoom level of the camera
        float lastFrameTime = 0.0f;    ///< Time of the last frame, used for frame time calculation

    public:
        friend class Mouse; // Allow Mouse class to access private members of Window

        /**
         * @brief Constructs a window with specified width, height, title, and other optional parameters.
         * @param width The width of the window in pixels.
         * @param height The height of the window in pixels.
         * @param title The title of the window.
         * @param bgcolor The background color of the window (default is black).
         * @param zoom The initial zoom level (default is 1.0).
         * @param antiAliasingLevel The level of anti-aliasing to apply (default is 8).
         */
        Window(unsigned int width, unsigned int height, const std::string& title, Graphics::Color bgcolor = Graphics::Color::Black,
            float zoom = 1.0f, int antiAliasingLevel = 8) : bgColor(bgcolor), zoomLevel(zoom) {

            sf::ContextSettings settings;
            settings.antialiasingLevel = antiAliasingLevel;  ///< Set anti-aliasing level

            // Create the window with anti-aliasing context settings
            window.create(sf::VideoMode(width, height), title, sf::Style::Default, settings);

            // Set up the camera view with the specified dimensions
            camera = sf::View(sf::FloatRect(
                -width / (2.0f * pixelPerMeter), // Left
                -height / (2.0f * pixelPerMeter), // Top
                width / pixelPerMeter,           // Width
                height / pixelPerMeter           // Height
            ));

            // Flip the y-axis
            camera.setSize(camera.getSize().x, -camera.getSize().y);

            // Set the camera center to the origin
            camera.setCenter(0.0f, 0.0f);

            // Apply the zoom level to the camera
            camera.zoom(zoomLevel);

            std::cout << "Zoom Level: " << zoomLevel << std::endl;

            // Set the window's frame rate limit and clear the screen with the background color
            window.setFramerateLimit(60);
            window.clear(bgColor.toSFML());
            window.setView(camera);
        }

        /**
         * @brief Polls for an event in the window.
         * @param event The event object to store event data.
         * @return True if an event is polled, false otherwise.
         */
        bool pollEvent(sf::Event& event) {
            return window.pollEvent(event);
        }

        /**
         * @brief Checks if any key is pressed.
         * @return True if any key is pressed, false otherwise.
         */
        bool isKeyPressed() {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::KeyPressed) {
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief Checks if the window is open.
         * @return True if the window is open, false otherwise.
         */
        bool isOpen() {
            return window.isOpen();
        }

        /**
         * @brief Closes the window.
         */
        void close() {
            window.close();
        }

        /**
         * @brief Clears the window with the current background color.
         */
        void clear() {
            window.clear(bgColor.toSFML());
        }

        /**
         * @brief Clears the window with the specified background color.
         * @param color The color to clear the window with.
         */
        void clear(Graphics::Color color) {
            window.clear(color.toSFML());
        }

        /**
         * @brief Changes the window's background color and clears it.
         * @param color The new background color.
         */
        void setWindowColor(Graphics::Color color) {
            bgColor = color;
            window.clear(bgColor.toSFML());
        }

        /**
         * @brief Displays the current contents of the window.
         */
        void display() {
            window.display();
        }

        /**
         * @brief Draws a shape to the window.
         * @param shape The shape to be drawn.
         */
        void draw(Shape& shape) {
            window.draw(shape.getShape());
        }

        /**
         * @brief Draws a circle with a border to the window.
         * @param radius The radius of the circle.
         * @param position The position of the circle.
         * @param borderColor The color of the border (default is transparent).
         * @param borderThickness The thickness of the border (default is 0).
         * @param fillColor The fill color of the circle (default is transparent).
         */
        void drawCircleWithBorder(float radius, const sf::Vector2f& position,
            Graphics::Color borderColor = Graphics::Color::Transparent, float borderThickness = 0,
            Graphics::Color fillColor = Graphics::Color::Transparent)
        {
            Graphics::Circle circle(radius, position.x, position.y, borderColor, borderThickness, fillColor);
            draw(circle);
        }

        /**
         * @brief Draws a filled circle to the window.
         * @param radius The radius of the circle.
         * @param position The position of the circle.
         * @param fillColor The fill color of the circle.
         */
        void drawCircleFilled(float radius, const sf::Vector2f& position, Graphics::Color fillColor)
        {
            Graphics::Circle circle(radius, position.x, position.y, Graphics::Color::Transparent, 0, fillColor);
            draw(circle);
        }

        /**
         * @brief Draws a rectangle with a border to the window.
         * @param width The width of the rectangle.
         * @param height The height of the rectangle.
         * @param angle The rotation angle of the rectangle.
         * @param position The position of the rectangle.
         * @param borderColor The color of the border (default is white).
         * @param borderThickness The thickness of the border (default is 0.1).
         * @param fillColor The fill color of the rectangle (default is transparent).
         */
        void drawRectangleWithBorder(float width, float height, float angle, const sf::Vector2f& position,
            Graphics::Color borderColor = Graphics::Color::White,
            float borderThickness = 0.1f, Graphics::Color fillColor = Graphics::Color::Transparent)
        {
            Graphics::Rectangle rectangle(width, height, angle, position, borderColor, borderThickness, fillColor);
            draw(rectangle);
        }

        /**
         * @brief Draws a filled rectangle to the window.
         * @param width The width of the rectangle.
         * @param height The height of the rectangle.
         * @param angle The rotation angle of the rectangle.
         * @param position The position of the rectangle.
         * @param fillColor The fill color of the rectangle.
         */
        void drawRectangleFilled(float width, float height, float angle, const sf::Vector2f& position, Graphics::Color fillColor)
        {
            Graphics::Rectangle rectangle(width, height, angle, position, Graphics::Color::Transparent, 0, fillColor);
            draw(rectangle);
        }

        /**
         * @brief Draws a polygon with a border to the window.
         * @param vertices A vector of vertices defining the polygon.
         * @param borderColor The color of the border (default is transparent).
         * @param borderThickness The thickness of the border (default is 0).
         * @param fillColor The fill color of the polygon (default is transparent).
         */
        void drawPolygonWithBorder(const std::vector<sf::Vector2f>& vertices,
            Graphics::Color borderColor = Graphics::Color::Transparent,
            float borderThickness = 0, Graphics::Color fillColor = Graphics::Color::Transparent)
        {
            Graphics::Polygon polygon(vertices, borderColor, borderThickness, fillColor);
            draw(polygon);
        }

        /**
         * @brief Draws a filled polygon to the window.
         * @param vertices A vector of vertices defining the polygon.
         * @param color The fill color of the polygon.
         */
        void drawPolygonFilled(const std::vector<sf::Vector2f>& vertices, Graphics::Color color) {
            Graphics::Polygon polygon(vertices, Graphics::Color::Transparent, 0, color);
            draw(polygon);
        }

        /**
         * @brief Draws a line between two points to the window.
         * @param point1 The start point of the line.
         * @param point2 The end point of the line.
         * @param color The color of the line (default is white).
         */
        void drawLine(const sf::Vector2f& point1, const sf::Vector2f& point2, Graphics::Color color = Graphics::Color::White) {
            Graphics::Line line(point1, point2, color);
            draw(line);
        }

        /**
         * @brief Draws a grid with lines at regular intervals.
         * @param gridSize The distance between two consecutive grid lines (default is 1.0f).
         * @param color The color of the grid lines (default is Graphics::Color::White).
         */
        void drawGridLines(float gridSize = 1.0f, Graphics::Color color = Graphics::Color::White) {
            float left, right, top, bottom;
            getCameraExtent(left, right, top, bottom);

            for (float x = 0; x <= right; x += gridSize) {
                drawLine(sf::Vector2f(x, top), sf::Vector2f(x, bottom), color);
                if (x != 0) {
                    drawLine(sf::Vector2f(-x, top), sf::Vector2f(-x, bottom), color);
                }
            }

            for (float y = 0; y <= top; y += gridSize) {
                drawLine(sf::Vector2f(left, y), sf::Vector2f(right, y), color);
                if (y != 0) {
                    drawLine(sf::Vector2f(left, -y), sf::Vector2f(right, -y), color);
                }
            }

            drawCircleFilled(0.1f, sf::Vector2f(0.0f, 0.0f), Graphics::Color::Red);
        }

        /**
         * @brief Writes text to the window at a specified position.
         * @param text The text to be written.
         * @param position The position where the text will be drawn.
         * @param color The color of the text (default is white).
         * @param size The size of the text (default is 24).
         * @param setOriginToCenter Whether to set the text origin to the center (default is true).
         * @param scale The scale factor to apply to the text (default is -1.0, no scaling).
         */
        void writeText(const std::string& text, const sf::Vector2f& position,
            Graphics::Color color = Graphics::Color::White, int size = 24, bool setOriginToCenter = true, float scale = -1.0f)
        {
            Graphics::Text textObj(text, position, color, size, setOriginToCenter);
            if (scale > 0) textObj.setScale(scale);
            draw(textObj);
        }

        /**
         * @brief Applies zoom to the camera.
         * @param factor The zoom factor.
         */
        void zoom(float factor) {
            zoomLevel *= factor;
            camera.zoom(factor);
            window.setView(camera);
        }

        /**
         * @brief Gets the camera's extents (left, right, top, and bottom boundaries).
         * @param left The left boundary of the camera's view.
         * @param right The right boundary of the camera's view.
         * @param top The top boundary of the camera's view.
         * @param bottom The bottom boundary of the camera's view.
         */
        void getCameraExtent(float& left, float& right, float& top, float& bottom) {
            sf::Vector2f size = camera.getSize();
            sf::Vector2f center = camera.getCenter();

            left = center.x - size.x / 2.0f;
            right = center.x + size.x / 2.0f;
            top = center.y - size.y / 2.0f;
            bottom = center.y + size.y / 2.0f;
        }

        /**
         * @brief Gets the bottom boundary of the camera's view.
         * @return The bottom boundary of the camera's view.
         */
        float getCameraBottom() {
            return camera.getCenter().y + camera.getSize().y / 2.0f;
        }

        /**
         * @brief Gets the top boundary of the camera's view.
         * @return The top boundary of the camera's view.
         */
        float getCameraTop() {
            return camera.getCenter().y - camera.getSize().y / 2.0f;
        }

        /**
         * @brief Gets the left boundary of the camera's view.
         * @return The left boundary of the camera's view.
         */
        float getCameraLeft() {
            return camera.getCenter().x - camera.getSize().x / 2.0f;
        }

        /**
         * @brief Gets the right boundary of the camera's view.
         * @return The right boundary of the camera's view.
         */
        float getCameraRight() {
            return camera.getCenter().x + camera.getSize().x / 2.0f;
        }

        /**
         * @brief Gets the elapsed time since the window was created or last reset.
         * @param unit The time unit to return (default is seconds).
         * @return The elapsed time in the specified unit.
         */
        float getElapsedTime(TimeUnit unit = TimeUnit::Seconds) {
            switch (unit) {
            case TimeUnit::Milliseconds:
                return clock.getElapsedTime().asMilliseconds();
            case TimeUnit::Seconds:
            default:
                return clock.getElapsedTime().asSeconds();
            }
        }

        /**
         * @brief Calculates the elapsed time since the last frame in the specified unit.
         * @param unit The unit in which the time should be returned (default is TimeUnit::Seconds).
         * @return The elapsed time since the last frame in the specified unit.
         */
        float getElapsedTimeSinceLastFrame(TimeUnit unit = TimeUnit::Seconds) const {
            float currentTime = 0.0f;

            switch (unit) {
            case TimeUnit::Milliseconds:
                currentTime = clock.getElapsedTime().asMilliseconds();
                break;
            case TimeUnit::Seconds:
            default:
                currentTime = clock.getElapsedTime().asSeconds();
                break;
            }

            return currentTime - lastFrameTime;
        }


        /**
         * @brief Updates the last frame time using the clock's elapsed time.
         * @params None
         */
        void update() {
            lastFrameTime = clock.getElapsedTime().asSeconds();
        }


        /**
         * @brief Resets the clock to start measuring elapsed time from now.
         */
        void resetClock() {
            clock.restart();
        }
    };
}

#endif // WINDOW_HPP
