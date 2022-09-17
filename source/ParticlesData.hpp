#pragma once

#include <vector>
#include "PBDVerletSolver.hpp"
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
    virtual ErrorCode addParticlesData(PBDVerletSolver* engine) const = 0;
};


class AABBParticlesData : public IParticlesData, AABB {
private:
    //glm::vec3 mPosition = {0, 0, 0};
    glm::vec4 mColor = {1, 1, 1, 1};
public:
    AABBParticlesData(glm::vec3 min, glm::vec3 max, glm::vec4 color): AABB(min, max), mColor(color) {}
    virtual ErrorCode addParticlesData(PBDVerletSolver* engine) const {
        ParticlesDataPayload result;

        AABB adjustedAABB = AABB(this->min, this->max);
        adjustedAABB.clampTo(engine->mAABB);

        glm::vec3 dims = adjustedAABB.max - adjustedAABB.min;
        if (dims.x <= 0 || dims.y <= 0 || dims.z <= 0) {
            return ELOY_ERROR_DOMAIN;
        }

        glm::vec3 particles_dims_f = dims / engine->mParticleDiameter;
        glm::ivec3 particles_dim = { static_cast<int>(particles_dims_f.x) + 1,  static_cast<int>(particles_dims_f.y) + 1, static_cast<int>(particles_dims_f.z) + 1 };
        
        int numNewParticles = particles_dim.x * particles_dim.y * particles_dim.z;
        if (numNewParticles <= 0) {
            return ELOY_ERROR_DOMAIN;
        }

        int particlePointer = engine->mNumParticles;
        engine->mNumParticles += numNewParticles;
        engine->resize(engine->mNumParticles);

        float invDiameter = 1.0f / engine->mParticleDiameter;
        for (int y = 0; y < particles_dim.y; y++) {
            for (int z = 0; z < particles_dim.z; z++) {
                for (int x = 0; x < particles_dim.x; x++) {
                    glm::vec3 newRelPosition = glm::vec3(x, y, z);
                    newRelPosition *= engine->mParticleDiameter;
                    glm::vec3 newBasePosition = adjustedAABB.min;
                    engine->mPositions[particlePointer] = newBasePosition + newRelPosition;
                    engine->mColors[particlePointer] = mColor;
                    particlePointer++;
                }
            }
        }

        return ELOY_ERROR_OK; 
    }
};

}