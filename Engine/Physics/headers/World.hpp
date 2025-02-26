#ifndef WORLD_HPP
#define WORLD_HPP

#include <vector>
#include <memory>

namespace Physics {

    class Body;

    class World {
        std::vector<std::shared_ptr<Body>> bodies;

    private:
        void calculateBodyAccelerations();

    public:
        void addBody(std::shared_ptr<Body> body);
        std::shared_ptr<Body> getBody(int index);
        void removeBody(std::shared_ptr<Body> body);

        size_t numBodies() const;

        void step(double time); // Orbit Mechanics

    };

} // namespace Physics

#endif // WORLD_HPP