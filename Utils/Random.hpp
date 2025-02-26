#ifndef RANDOM_HPP
#define RANDOM_HPP

#include <random>
#include "../Graphics/Color.hpp"

namespace Utils {
    /**
     * @class Random
     * @brief Provides static methods for generating random values (float, int, bool, and color).
     */
    class Random {
    public:
        // Random Number Generator
        static std::mt19937 randomEngine;

        /**
         * @brief Generates a random float value between the specified minimum and maximum values.
         * @param min The minimum value for the random float.
         * @param max The maximum value for the random float.
         * @return A random float between min and max.
         */
        static float Float(float min, float max) {
            std::uniform_real_distribution<float> dist(min, max);
            return dist(randomEngine);
        }

        /**
         * @brief Generates a random integer value between the specified minimum and maximum values.
         * @param min The minimum value for the random integer.
         * @param max The maximum value for the random integer.
         * @return A random integer between min and max.
         */
        static int Int(int min, int max) {
            std::uniform_int_distribution<int> dist(min, max);
            return dist(randomEngine);
        }

        /**
         * @brief Generates a random boolean value (true or false).
         * @return A random boolean value.
         */
        static bool Bool() {
            return Int(0, 1) == 1;
        }

        /**
         * @brief Generates a random color by generating random RGB values.
         * @return A random color with random red, green, and blue components.
         */
        static Graphics::Color Color() {
            return Graphics::Color(Int(0, 255), Int(0, 255), Int(0, 255));
        }
    };

    std::mt19937 Random::randomEngine(std::random_device{}());

} // namespace Utils

#endif // RANDOM_HPP
