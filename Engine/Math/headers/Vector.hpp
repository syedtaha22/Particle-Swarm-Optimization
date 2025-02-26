#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <iostream>
#include <string>

namespace Math {

    // Forward Declaration of Transformation
    struct Transformation;

    struct Vector {
        double x, y;

        // Define zero vector
        static Vector Zero;

        Vector(double x = 0.0f, double y = 0.0f);


        Vector operator+(const Vector& other) const; // Addition
        Vector operator-(const Vector& other) const; // Subtraction
        Vector operator*(double scalar) const; // Multiplication
        Vector operator/(double scalar) const; // Division

        friend Vector operator*(double scalar, const Vector& vector); // Multiplication

        Vector& operator+=(const Vector& other); // In-place addition
        Vector& operator-=(const Vector& other); // In-place subtraction
        Vector& operator*=(double scalar); // In-place multiplication
        Vector& operator/=(double scalar); // In-place division

        Vector operator-() const; // Negation
        bool operator==(const Vector& other) const; // Check for equality
        std::string toString() const; // To string

        // Static function to transform a vector
        static Vector Transform(const Vector& vector, Transformation& transform);

        // hash function
        size_t hash() const;


    };

} // namespace Math


#endif // VECTOR_HPP