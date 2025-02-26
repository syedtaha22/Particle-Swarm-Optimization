#include <algorithm>

#include "../headers/World.hpp"
#include "../headers/Body.hpp"
#include "../headers/Constants.hpp"
#include "../headers/Property.hpp"

#include "../../Math/headers/Operation.hpp"
#include "../../Math/headers/Vector.hpp"


namespace Physics {

    void World::addBody(std::shared_ptr<Body> body) {
        bodies.push_back(body);
    }

    void World::removeBody(std::shared_ptr<Body> body) {
        auto it = std::find(bodies.begin(), bodies.end(), body);
        if (it != bodies.end()) {
            bodies.erase(it);
        }
    }

    std::shared_ptr<Body> World::getBody(int index) {
        if (index >= 0 && index < bodies.size()) {
            return bodies[index];
        }
        throw std::out_of_range("Index out of range");
    }

    size_t World::numBodies() const {
        return bodies.size();
    }

    void World::step(double time) {
        calculateBodyAccelerations();
        for (size_t i = 0; i < bodies.size(); i++) {
            bodies[i]->step(time);
        }
    }

    void World::calculateBodyAccelerations() {
        for (size_t i = 0; i < bodies.size(); i++) {
            bodies[i]->setKinematicProperty(KinematicProperty::Acceleration, Math::Vector::Zero);

            Math::Vector position =
                bodies[i]->getKinematicProperty(KinematicProperty::Position);

            for (size_t j = 0; j < bodies.size(); j++) {
                if (i == j) continue;

                Math::Vector direction =
                    bodies[j]->getKinematicProperty(KinematicProperty::Position) - position;

                double distance = Math::Operation::Length(direction);
                direction = direction / distance;

                double mass = bodies[j]->getPhysicalProperty(PhysicalProperty::Mass);

                double accelerationMagnitude = Constants::GRAVITATIONAL_CONSTANT * mass / (distance * distance);
                Math::Vector acceleration = direction * accelerationMagnitude;

                bodies[i]->addKinematicProperty(KinematicProperty::Acceleration, acceleration);
            }
        }
    }

} // namespace Physics