#pragma once

#include <vector>
#include "Engine.hpp"
#include "glm/glm.hpp"

namespace Eloy {

struct ParticlesDataPayload {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec4> colors;
};

class IParticlesData {
public:
    virtual ParticlesDataPayload getParticlesData(Engine* engine) = 0;
};
}