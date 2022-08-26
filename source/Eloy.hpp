
#include "Engine.hpp"
#include "EngineParameters.hpp"
#include "AABB.hpp"
#include "ParticlesData.hpp"
#include "ParticlesRendering.hpp"
#include "EngineImGui.hpp"

#include <utility>

namespace Eloy {
    extern std::pair<std::vector<glm::vec3>, std::vector<glm::vec4>> getParticlesStateFromSaveJson();
}