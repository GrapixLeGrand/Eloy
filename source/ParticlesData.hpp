#pragma once

#include <vector>
#include "Engine.hpp"
#include "glm/glm.hpp"
#include "Error.hpp"
#include "AABB.hpp"

namespace Eloy {

struct ParticlesDataPayload {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec4> colors;
    ErrorCode error;
};

struct ParticlesDataTransform {
    glm::vec3 position;
    glm::mat3 basis;
};

class IParticlesData {
    //ParticlesDataTransform transform;
public:
    virtual ErrorCode addParticlesData(Engine* engine) const = 0;
};


class AABBParticlesData : public IParticlesData, AABB {
private:
    glm::vec3 mPosition = {0, 0, 0};
public:
    AABBParticlesData(glm::vec3 position, glm::vec3 min, glm::vec3 max): AABB(min, max), mPosition(position) {}
    virtual ErrorCode addParticlesData(Engine* engine) const {
        ParticlesDataPayload result;

        AABB adjustedAABB = AABB(this->min, this->max);
        adjustedAABB.clampTo(engine->mAABB);

        glm::vec3 dims = adjustedAABB.max - adjustedAABB.min;
        if (dims.x <= 0 || dims.y <= 0 || dims.z <= 0) {
            return ELOY_ERROR_DOMAIN;
        }

        glm::vec3 particles_dims_f = dims / engine->mParticleDiameter;
        glm::ivec3 particles_dim = { static_cast<int>(dims.x) + 1,  static_cast<int>(dims.y) + 1, static_cast<int>(dims.z) + 1 };

        for (int y = 0; y < particles_dim.y; y++) {
            for (int z = 0; z < particles_dim.z; z++) {
                for (int x = 0; x < particles_dim.x; x++) {
                    
                }
            }
        }

        return ELOY_ERROR_OK; 
    }
};

}