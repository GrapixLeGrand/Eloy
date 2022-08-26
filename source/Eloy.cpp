
#include "Eloy.hpp"
#include "nlohmann/json.hpp"

namespace Eloy {

    extern std::pair<std::vector<glm::vec3>, std::vector<glm::vec4>> getParticlesStateFromSaveJson() {
        
        std::ifstream particlesfile(ELOY_BUILD_DIRECTORY"/save.json");
        nlohmann::json particlesJson = nlohmann::json::parse(particlesfile);

        std::vector<std::vector<float>> particles = particlesJson["positions"];
        std::vector<std::vector<float>> colors = particlesJson["colors"];
        std::pair<std::vector<glm::vec3>, std::vector<glm::vec4>> result;
        result.first.reserve(particles.size());
        for (auto& p : particles) {
            result.first.push_back({p[0], p[1], p[2]});
        }

        result.second.reserve(colors.size());
        for (auto& p : colors) {
            result.second.push_back({p[0], p[1], p[2], p[3]});
        }
        return result;
    }

}