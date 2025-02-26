#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP

namespace Math {

    // Forward Declaration of FlatVector
    struct Vector;

    struct Transformation {
        const double PositionX;
        const double PositionY;
        const double sine;
        const double cosine;

        // Static Zero transformation
        static const Transformation Zero;

        Transformation(const Vector& position, double angle);

        Transformation(double x, double y, double angle);
    };



} // namespace Math




#endif // TRANSFORMATION_HPP