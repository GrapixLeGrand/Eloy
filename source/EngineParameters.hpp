#pragma once

#include <vector>
#include "glm/glm.hpp"
#include "ParticlesData.hpp"

namespace Eloy {
class EngineParameters {
public:
    int mX = 20, mY = 20, mZ = 20;
    float mParticuleRadius          = static_cast<float>(0.25);
    float mKernelFactor             = static_cast<float>(1.5);
    float mRestDensity              = static_cast<float>(24.0);
    float mMass                     = static_cast<float>(2.5);
    float mTimeStep                 = static_cast<float>(0.01);
    glm::vec3 mGravity              = glm::vec3(0, -10.0, 0.0);
    float mRelaxationEpsilon        = static_cast<float>(10.0);
    float mSCorrDeltaQ              = static_cast<float>(0.5);
    float mSCorrK                   = static_cast<float>(1.0);
    float mSCorrN                   = static_cast<float>(4);
    float mCXsph                    = static_cast<float>(0.1);
    float mEpsilonVorticity         = static_cast<float>(0.1);

    std::vector<IParticlesData*> mParticlesData;
};

}