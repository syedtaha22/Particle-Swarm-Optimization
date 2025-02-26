
#include "../headers/Path.hpp"

#include "../../Math/headers/Vector.hpp"

namespace Physics {

    void Path::insert(const Math::Vector& v) {
        size_t hash = v.hash();

        if (hashToVector.find(hash) == hashToVector.end()) {
            hashList.push_back(hash);
            hashToVector[hash] = v;
        }
    }

    Math::Vector Path::get(int i) const {
        if (i < 0 || i >= hashList.size()) {
            throw std::out_of_range("Index out of range");
        }
        size_t hash = hashList[i];

        return hashToVector.at(hash);
    }

    size_t Path::getSize() const {
        return hashList.size();
    }
} // namespace Physics
