#ifndef MATH_CONSTANTS_HPP
#define MATH_CONSTANTS_HPP


namespace Math {

    class Constants {
    public:
        static constexpr double PI = 3.141592653589793;        // Ratio of a circle's circumference to its diameter
        static constexpr double e = 2.718281828459045;         // Base of natural logarithm
        static constexpr double TAU = 6.283185307179586;       // Tau (2 * Pi), useful for circles

        static constexpr double DoubleMax = 1.797693e308;
        static constexpr double DoubleMin = -1.797693e308;
    };

} // namespace Math




#endif // MATH_CONSTANTS_HPP