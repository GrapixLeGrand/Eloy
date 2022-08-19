#pragma once

#include "ParticlesData.hpp"
#include "glm/glm.hpp"

namespace Eloy {
class AABB : public IParticlesData {
public:
    
    AABB() {}
    AABB(glm::vec3 min, glm::vec3 max)
    :
    min(min),
    max(max) {};

    glm::vec3 min;
    glm::vec3 max;
    
};
}