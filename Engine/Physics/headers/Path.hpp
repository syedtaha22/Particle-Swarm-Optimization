#ifndef PATH_HPP
#define PATH_HPP

#include <vector>
#include <unordered_map>

namespace Math {
    struct Vector;
}

namespace Physics {

    class Path {
    private:
        std::vector<size_t> hashList;  // List of hashes for the order of insertion
        std::unordered_map<size_t, Math::Vector> hashToVector;  // Map from hash to vector

    public:
        // Insert a Position
        void insert(const Math::Vector& v);

        // Get the i-th position in the path
        Math::Vector get(int i) const;

        // Get the size of the path
        size_t getSize() const;
    };

} // namespace Physics




#endif // PATH_HPP