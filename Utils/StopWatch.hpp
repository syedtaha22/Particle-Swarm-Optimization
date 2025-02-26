#ifndef STOPWATCH_HPP
#define STOPWATCH_HPP

#include <chrono>
#include <thread>

namespace Utils {

    /**
     * @class Stopwatch
     * @brief Stopwatch class for measuring elapsed time in seconds or milliseconds.
     * It allows starting, stopping, resetting, and restarting the stopwatch, as well as retrieving the elapsed time.
     */
    class Stopwatch {
    private:
        std::chrono::steady_clock::time_point start_time; ///< The start time of the stopwatch.
        std::chrono::steady_clock::time_point end_time; ///< The end time of the stopwatch.
        bool running; ///< Indicates whether the stopwatch is currently running.

    public:
        /**
         * @brief Constructs a Stopwatch object, initializing it as stopped.
         */
        Stopwatch() : running(false) {}

        /**
         * @brief Starts the stopwatch.
         * If the stopwatch is not already running, it captures the start time and marks it as running.
         */
        void start() {
            if (!running) {
                start_time = std::chrono::steady_clock::now();
                running = true;
            }
        }

        /**
         * @brief Stops the stopwatch.
         * If the stopwatch is currently running, it captures the end time and marks it as stopped.
         */
        void stop() {
            if (running) {
                end_time = std::chrono::steady_clock::now();
                running = false;
            }
        }

        /**
         * @brief Resets the stopwatch.
         * This stops the stopwatch and sets both the start and end times to 0.
         */
        void reset() {
            running = false;
            start_time = std::chrono::steady_clock::time_point(); ///< Set start time to 0.
            end_time = std::chrono::steady_clock::time_point();   ///< Set end time to 0.
        }

        /**
         * @brief Restarts the stopwatch.
         * This resets the stopwatch and then immediately starts it again.
         */
        void restart() {
            reset();
            start();
        }

        /**
         * @brief Gets the elapsed time in seconds.
         * @return The elapsed time in seconds since the stopwatch started or stopped.
         */
        double getElapsedTimeInSeconds() {
            if (running) {
                return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            }
            return std::chrono::duration<double>(end_time - start_time).count();
        }

        /**
         * @brief Gets the elapsed time in milliseconds.
         * @return The elapsed time in milliseconds since the stopwatch started or stopped.
         */
        double getElapsedTimeInMilliseconds() {
            if (running) {
                return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
            }
            return std::chrono::duration<double, std::milli>(end_time - start_time).count();
        }
    };

} // namespace Utils

#endif // STOPWATCH_HPP
