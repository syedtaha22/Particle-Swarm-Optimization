#ifndef BODY_HPP
#define BODY_HPP

#include <unordered_map>
#include <string>
#include <memory>

// Forward Declaration of Math::Vector
namespace Math {
    struct Vector;
}

namespace Physics {

    // Forward Declaration 
    enum class PhysicalProperty;
    enum class KinematicProperty;

    class Body {

        // Physical Properties
        std::unordered_map<PhysicalProperty, double> physicalProperties;

        // Kinematic Properties
        std::unordered_map<KinematicProperty, Math::Vector> kinematicProperties;

    public:
        Body(double mass, const Math::Vector& position);

        // Setters
        void setPhysicalProperty(PhysicalProperty property, double value);
        void setKinematicProperty(KinematicProperty property, const Math::Vector& value);

        // Adders
        void addPhysicalProperty(PhysicalProperty property, double value);
        void addKinematicProperty(KinematicProperty property, const Math::Vector& value);

        // Getters
        double getPhysicalProperty(PhysicalProperty property) const;
        Math::Vector getKinematicProperty(KinematicProperty property) const;

        // Existence Checkers
        bool physicalPropertyExists(PhysicalProperty property) const;
        bool kinematicPropertyExists(KinematicProperty property) const;

        // Step the body
        void step(double time);
    };

} // namespace Physics


#endif // BODY_HPP