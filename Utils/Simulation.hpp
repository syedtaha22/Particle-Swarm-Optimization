#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>     // For storing vectors of colors and paths
#include <memory>     // For std::shared_ptr
#include <fstream>    // For reading CSV files
#include <sstream>    // For parsing CSV data
#include <iostream>   // For error output

// Include Graphics Core for rendering and user interactions
#include "../Graphics/core.hpp"

// Include Physics Core for simulating physics
#include "../Engine/Physics/core.hpp"

// Include Math Core for mathematical operations
#include "../Engine/Math/core.hpp"

#include "Random.hpp" // For generating random colors

namespace Utils {

    /**
     * @brief Simulation class for simulating a simple solar system with physics, rendering, and user interactions.
     */
    class Simulation {
    protected:
        Graphics::Window window; ///< Window object for rendering and handling user interactions
        std::string scaleText; ///< Text to display scale information
        bool started = false; ///< Flag to toggle simulation start/stop
        bool showGrid = true; ///< Flag to toggle grid display
        double speedFactor; ///< Speed factor for simulation

    private:

        /**
         * @brief Displays the scale and paused state information.
         */
        void displayScale() {
            float left = window.getCameraLeft();
            float top = window.getCameraTop();
            float padding = abs(top - window.getCameraBottom()) * 0.05f;

            if (!started) {
                window.writeText("Paused", sf::Vector2f(left + padding, top - padding),
                    Graphics::Color::White, 1, false, 0.7f); // Display "Paused" when simulation is stopped
            }

            float bottom = window.getCameraBottom();

            window.writeText(scaleText, sf::Vector2f(left + padding, bottom + padding),
                Graphics::Color::White, 1, false, 0.7f); // Display scale text
        }

    public:
        /**
         * @brief Constructs a Simulation object, initializing the window, physics, and other parameters.
         */
        Simulation(const std::string& title, unsigned int width = 1280, unsigned int height = 768,
            Graphics::Color bgColor = Graphics::Color::Black, float zoom = 5.0f, unsigned int gridSpacing = 32)
            : window(width, height, title, bgColor, zoom, gridSpacing) {
            speedFactor = 1e7; // Set the speed factor for the simulation
            // Load a font for text rendering
            Graphics::Text::loadFont("Resources/Fonts/Roboto-Regular.ttf");
        }

        /**
         * @brief Virtual function to get the maximum distance from the origin.
         * @paragraph This function must be overridden by the user to provide the simulation with
         * the maximum distance, which is used to set the scale for the simulation.
         * @return Maximum distance from the origin.
         */
        virtual double get_max_distance() = 0;

        /**
         * @brief Initializes the simulation with celestial bodies from a CSV file.
         */
        void init() {
            double maxDistance = get_max_distance();

            // Get the order of magnitude of the maximum distance
            double scale = std::pow(10, static_cast<int>(std::log10(maxDistance)) - 1);
            Math::Converter::setScale(scale); // Set the scale for conversion

            // Update the scale text to display the scale information in scientific notation, using sstream
            std::ostringstream oss;
            oss << std::scientific << std::setprecision(0) << scale << " m";
            scaleText = oss.str();
        }

        /**
         * @brief Sets the speed factor for the simulation.
         * @param factor Speed factor value.
         */
        void setSpeed(double factor) {
            Math::Operation::Clamp(factor, 0.0, 15.0); // Clamp the factor between 0 and 15
            speedFactor = std::pow(10, factor); // Set the speed factor for the simulation
        }

        /**
         * @brief Updates the simulation state, processing events and advancing the simulation.
         */
        void update() {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) window.close();

                // Handle zoom in/out
                if (event.type == sf::Event::MouseWheelScrolled) {
                    if (event.mouseWheelScroll.delta > 0) window.zoom(0.95f); // Zoom in
                    else window.zoom(1.05f); // Zoom out
                }

                if (event.type == sf::Event::KeyPressed) {
                    // Toggle simulation start/stop
                    if (event.key.code == sf::Keyboard::Space) started = !started;
                    // Handle grid toggle with G key
                    if (event.key.code == sf::Keyboard::G) showGrid = !showGrid;
                }
            }

            if (started) step(); // Step through the simulation if started
            window.update(); // Update the window
        }


        /**
         * @brief Steps through the simulation logic provided by the user.
         * @paragraph This function should be overridden by the user to define the simulation logic.
         */
        virtual void step() = 0;

        /**
         * @brief Virtual method to draw the objects in the simulation. Must be overridden by the user.
         * @paragraph This function should be overridden by the user to define the drawing logic.
         */
        virtual void draw_bodies() = 0;

        /**
         * @brief Draws all elements of the simulation, including bodies, orbits, and scale.
         * @paragraph This function can be overridden by the user for more control over the drawing process.
         */
        virtual void draw() {
            window.clear(); // Clear the window

            // Draw grid lines
            if (showGrid) window.drawGridLines(1.0f, Graphics::Color("#333333"));

            // Draw celestial bodies
            draw_bodies();

            displayScale(); // Display scale text
            window.display(); // Display everything on the window
        }

        /**
         * @brief Checks if the window is open.
         * @return True if the window is open, false otherwise.
         */
        bool isOpen() {
            return window.isOpen();
        }
    };
}

#endif // SIMULATION_HPP