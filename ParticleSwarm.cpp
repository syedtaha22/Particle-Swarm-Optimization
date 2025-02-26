#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

#include "./Utils/Simulation.hpp"

/**
 * @brief Function class to represent the function sin(x) - sin(3x).
 */
class Function {
    std::vector<Math::Vector> data;
public:
    /**
     * @brief Construct a new Function object with the specified number of data points.
     * @param n Number of data points to generate.
     */
    Function(int n = 100) {
        float offset = 8.0 / n;
        for (float x = -4; x <= 4; x += offset) data.push_back(Math::Vector(x, operator()(x)));
    }

    /**
     * @brief Draws the function on the specified window.
     * @param window Window to draw the function on.
     */
    void draw(Graphics::Window& window) {
        for (int i = 0; i < data.size() - 1; i++) {
            window.drawLine(
                Math::Converter::toVector2f(data[i]),
                Math::Converter::toVector2f(data[i + 1]),
                Graphics::Color::White);
        }
    }

    /**
     * @brief Function operator to calculate the value of the function at the specified x value.
     * @param x X value to calculate the function value at.
     * @return Value of the function at the specified x value.
     */
    constexpr float operator()(float x) const { return sin(x) - sin(3 * x); }

    /**
     * @brief Get the error value for the specified x value.
     * @param x X value to calculate the error for.
     * @return Error value for the specified x value.
     */
    static float GetError(float x) { return fabs(2 - Function()(x)); }
};

/**
 * @brief Particle class to represent a particle in the Particle Swarm Optimization algorithm.
 */
class Particle {
public:
    Math::Vector position;
    Math::Vector velocity;
    Math::Vector best_position;
    float best_value;

    /**
     * @brief Initializes a Particle with a random position and velocity.
     *
     * @note The position is randomly selected within [-4, 4] along the x-axis.
     * The initial velocity is zero, and the best-known position is set to the starting position.
     */
    Particle() {
        position = Math::Vector(Utils::Random::Float(-4, 4), Utils::Random::Float(-4, 4));
        best_value = Function()(position.x);
        velocity = Math::Vector(0.0, 0.0);
        best_position = Math::Vector(position.x, best_value);
    }
};

/**
 * @brief ParticleSwarm class to represent a swarm of particles in the Particle Swarm Optimization algorithm.
 */
class ParticleSwarm {
    std::vector<Particle> particles;
    Math::Vector global_best_position;
    float global_best_value;

    // Constants for the Particle Swarm
    const float w;
    const float c1;
    const float c2;
public:
    /**
     * @brief Construct a new Particle Swarm object with the specified swarm size and constants.
     * @param swarm_size Number of particles in the swarm.
     * @param _w Inertia weight for the swarm.
     * @param _c1 Cognitive weight for the swarm.
     * @param _c2 Social weight for the swarm.
     */
    ParticleSwarm(int swarm_size, float _w, float _c1, float _c2) : w(_w), c1(_c1), c2(_c2) {
        particles.resize(swarm_size);
        global_best_value = std::numeric_limits<float>::lowest();
        for (auto& p : particles) {
            if (p.best_value > global_best_value) {
                global_best_value = p.best_value;
                global_best_position = p.best_position;
                global_best_position.y = Function()(global_best_position.x);
            }
        }
    }

    /**
     * @brief Get the particle at the specified index.
     * @param i Index of the particle to get.
     * @return Reference to the particle at the specified index.
     */
    const Particle& get(int i) const { return particles[i]; }

    /**
     * @brief Steps through the Particle Swarm Optimization algorithm for the specified time.
     * @param time Time step for the simulation.
     */
    void step(double time) {
        for (auto& p : particles) {
            float r1 = Utils::Random::Float(0, 1);
            float r2 = Utils::Random::Float(0, 1);

            // V(t+1) = c1 * r1 * (P(t) - X(t)) + c2 * r2 * (G(t) - X(t))
            Math::Vector velocity_update =
                (c1 * r1) * (p.best_position - p.position) +
                (c2 * r2) * (global_best_position - p.position);
            p.velocity = p.velocity * w + velocity_update * time;
            p.position += p.velocity * time;

            float value = Function()(p.position.x);
            if (value > p.best_value) {
                p.best_value = value;
                p.best_position = Math::Vector(p.position.x, value);
            }
            if (p.best_value > global_best_value) {
                global_best_value = p.best_value;
                global_best_position = p.best_position;
            }
        }
    }
};

/**
 * @brief Particle Swarm Simulation class to simulate the Particle Swarm Optimization algorithm.
 *
 * @note Inherits from Utils::Simulation to provide a simulation interface.
 */
class PSS : public Utils::Simulation {
    ParticleSwarm swarm;
    Function func;
    const int swarm_size;

    /**
     * @brief Draws a box around the origin to represent the search space.
     */
    void drawBox() {
        for (int i = -1; i <= 1; i += 2) {
            window.drawLine(
                Math::Converter::toVector2f(Math::Vector(-4, i * 4)),
                Math::Converter::toVector2f(Math::Vector(4, i * 4)));
            window.drawLine(
                Math::Converter::toVector2f(Math::Vector(i * 4, -4)),
                Math::Converter::toVector2f(Math::Vector(i * 4, 4)));
        }
    }

    /**
     * @brief Get the maximum distance from the origin for the simulation.
     */
    double get_max_distance() override {
        double maxDistance = 0.0;
        for (int i = 0; i < swarm_size; i++) {
            double distance = Math::Operation::Length(swarm.get(i).position);
            if (distance > maxDistance) maxDistance = distance;
        }

        return maxDistance;
    }

    /**
     * @brief Steps through the simulation, updating the Particle Swarm Optimization algorithm.
     */
    void step() override {
        swarm.step(window.getElapsedTimeSinceLastFrame(Graphics::TimeUnit::Seconds));
    }

    /**
     * @brief Draws the bodies in the simulation, including the function and particles.
     */
    void draw_bodies() override {
        func.draw(window);
        // drawBox();

        for (int i = 0; i < swarm_size; i++) {
            Math::Vector pos = swarm.get(i).position;
            window.drawCircleFilled(0.07f, Math::Converter::toVector2f(pos), Graphics::Color::Red);
        }
    }

public:
    /**
     * @brief Construct a new Particle Swarm Simulation object with the specified swarm size and constants.
     * @param s_s Swarm size for the simulation.
     * @param w Inertia weight for the swarm.
     * @param c1 Cognitive weight for the swarm.
     * @param c2 Social weight for the swarm.
     */
    PSS(int s_s = 50, float w = 0.5, float c1 = 1.5, float c2 = 5.0) :
        Utils::Simulation("Particle Swarm Simulation"), swarm_size(s_s), swarm(s_s, w, c1, c2) {
        init();
        setSpeed(1);
        func = Function(500);
    }

    /**
     * @brief Updates the simulation by handling events and updating the window.
     */
    void draw() override {
        window.clear();
        draw_bodies();
        window.display();
    }
};



int main() {
    /*
        We rely on good values for
         - w (inertia weight) - How much the particle keeps moving in the same direction
         - c1 (cognitive weight) - How much the particle moves towards its best position
         - c2 (social weight) - How much the particle moves towards the global best position

        A good value for w is 0.95, c1 is 1, and c2 is 1
            - w = 0.95, not too slow, not to fast, prevents offshoots
            - c1 = c2 = 1, the particle moves towards the average of its best position and
             the global best position

        For experimentation,
            Keep c1 = c2 = 1, and increase w to 1.5.
                - Observe that the particles overshoot the global best position
                - Makes a very good effect, of particles offshoots, and then come back

            Keep c1 = 0, c2 = 1, and w = 0.5
                - Observe that the particles move towards the global best position

            Keep c1 = 1, c2 = 0, and w = 0.5
                - Observe that the particles move towards the best position of the particle
    */

    int swarm_size = 500;
    float w = 0.99;
    float c1 = 1;
    float c2 = 0;

    PSS simulation(swarm_size, w, c1, c2);

    while (simulation.isOpen()) {
        simulation.update();
        simulation.draw();
    }
}

