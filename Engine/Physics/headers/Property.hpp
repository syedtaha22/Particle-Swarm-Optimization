#ifndef PROPERTY_HPP
#define PROPERTY_HPP



namespace Physics {

    enum class PhysicalProperty {
        Mass,
        InverseMass,
    };

    enum class KinematicProperty {
        Position,
        LinearVelocity,
        Force,
        Acceleration
    };

}

#endif // PROPERTY_HPP