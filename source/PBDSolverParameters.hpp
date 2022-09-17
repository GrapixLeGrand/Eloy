#pragma once

#include <vector>
#include "glm/glm.hpp"
#include "ParticlesData.hpp"

namespace Eloy {
/*
class EngineParameters {
public:
    int mX = 20, mY = 20, mZ = 20;
    float mParticuleRadius          = static_cast<float>(0.25);
    float mKernelFactor             = static_cast<float>(0.1);
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
};*/


//worked well with the sim with the pressure uncorrected
class PBDSolverParameters {
public:
    int mX = 6, mY = 15, mZ = 6;
    float mParticuleRadius          = static_cast<float>(0.1);
    float mKernelRadius             = static_cast<float>(0.32);
    float mKernelFactor             = static_cast<float>(1);
    float mBoundaryCollisionCoeff   = static_cast<float>(1.2);
    float mRestDensity              = static_cast<float>(1000.0);
    float mMass                     = static_cast<float>(0.19);
    float mTimeStep                 = static_cast<float>(0.016);
    glm::vec3 mGravity              = glm::vec3(0, -10.0, 0.0);
    float mRelaxationEpsilon        = static_cast<float>(1e+05);
    float mSCorrDeltaQ              = static_cast<float>(0.022); //was 0.3 * 0.1
    float mSCorrK                   = static_cast<float>(0.029);
    float mSCorrN                   = static_cast<float>(5);//1.57
    float mCXsph                    = static_cast<float>(0.1);
    float mEpsilonVorticity         = static_cast<float>(0.175);
    int mSubsteps                   = 2;

    std::vector<IParticlesData*> mParticlesData;
};

/*
class EngineParameters {
public:
    int mX = 40, mY = 40, mZ = 30;
    float mParticuleRadius          = static_cast<float>(0.5);
    float mKernelFactor             = static_cast<float>(0.387);
    float mBoundaryCollisionCoeff   = static_cast<float>(0.9);
    float mRestDensity              = static_cast<float>(24.0);
    float mMass                     = static_cast<float>(5);
    float mTimeStep                 = static_cast<float>(0.01);
    glm::vec3 mGravity              = glm::vec3(0, -10.0, 0.0);
    float mRelaxationEpsilon        = static_cast<float>(16.0);
    float mSCorrDeltaQ              = static_cast<float>(0.5);
    float mSCorrK                   = static_cast<float>(1.0);
    float mSCorrN                   = static_cast<float>(4);
    float mCXsph                    = static_cast<float>(0.1);
    float mEpsilonVorticity         = static_cast<float>(0.1);
    int mSubsteps                   = 1;

    std::vector<IParticlesData*> mParticlesData;
};*/

}