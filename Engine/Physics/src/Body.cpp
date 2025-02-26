#include <iostream>
#include <stdexcept>


// Physics Headers
#include "../headers/Body.hpp"
#include "../headers/Property.hpp"

// Math Headers
#include "../../Math/headers/Vector.hpp"
#include "../../Math/headers/Operation.hpp"
#include "../../Math/headers/Constants.hpp"


// #include "../headers/Transformation.hpp"
// #include "../headers/Vector.hpp"
// #include "../headers/FlatTransformation.hpp"
// #include "../headers/FlatWorld.hpp"
// #include "../headers/FlatMath.hpp"
// #include "../headers/FlatAABB.hpp"

namespace Physics {


    Body::Body(double mass, const Math::Vector& position)
    {
        // Add Kinematic Properties
        kinematicProperties[KinematicProperty::Position] = position;
        kinematicProperties[KinematicProperty::LinearVelocity] = Math::Vector(0, 0);
        kinematicProperties[KinematicProperty::Force] = Math::Vector(0, 0);

        // Add Physical Properties
        physicalProperties[PhysicalProperty::Mass] = mass;
        physicalProperties[PhysicalProperty::InverseMass] = 1.0 / mass;
    }

    // Set Physical Property
    void Body::setPhysicalProperty(PhysicalProperty property, double value) {
        physicalProperties[property] = value;
    }

    void Body::addPhysicalProperty(PhysicalProperty property, double value) {
        physicalProperties[property] += value;
    }

    // Check if Physical Property exists
    bool Body::physicalPropertyExists(PhysicalProperty property) const {
        return physicalProperties.find(property) != physicalProperties.end();
    }

    // Get Physical Property
    double Body::getPhysicalProperty(PhysicalProperty property) const {
        if (physicalPropertyExists(property)) {
            return physicalProperties.at(property);
        }
        throw std::runtime_error("Physical Property does not exist");
    }

    // Set Kinematic Property
    void Body::setKinematicProperty(KinematicProperty property, const Math::Vector& value) {
        kinematicProperties[property] = value;
    }

    void Body::addKinematicProperty(KinematicProperty property, const Math::Vector& value) {
        kinematicProperties[property] += value;
    }

    // Check if Kinematic Property exists
    bool Body::kinematicPropertyExists(KinematicProperty property) const {
        return kinematicProperties.find(property) != kinematicProperties.end();
    }

    // Get Kinematic Property
    Math::Vector Body::getKinematicProperty(KinematicProperty property) const {
        if (kinematicPropertyExists(property)) {
            return kinematicProperties.at(property);
        }
        throw std::runtime_error("Kinematic Property does not exist");
    }

    void Body::step(double time) {
        // Update linear velocity
        kinematicProperties[KinematicProperty::LinearVelocity] +=
            kinematicProperties[KinematicProperty::Acceleration] * time;

        // Update position
        kinematicProperties[KinematicProperty::Position] +=
            kinematicProperties[KinematicProperty::LinearVelocity] * time;
    }

} // namespace Physics