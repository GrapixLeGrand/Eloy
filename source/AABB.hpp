#pragma once

#include "glm/glm.hpp"

namespace Eloy {
class AABB {
public:
    
    AABB() {}
    AABB(glm::vec3 min, glm::vec3 max)
    :
    min(min),
    max(max) {};

    glm::vec3 min;
    glm::vec3 max;

    void clampTo(AABB other) {
        min = glm::clamp(min, other.min, glm::vec3(std::numeric_limits<float>::max()));
        min = glm::clamp(max, glm::vec3(std::numeric_limits<float>::lowest()), other.max);
    }

};
}