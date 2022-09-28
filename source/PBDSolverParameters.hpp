#pragma once

#include <vector>
#include "glm/glm.hpp"
//#include "ParticlesData.hpp"

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


/**
 * @brief Notes
 * 
 * The kernel radius must be 3 * particle radius. Draw 9 balls in a box, 
 * you see that the side of the box is 3 radius of particles.
 * This must be the size of a cell.
 * 
 */

class IParticlesData;
//class PBDSolver;
//worked well with the sim with the pressure uncorrected
class PBDSolverParameters {
public:
    int mX = 3, mY = 3, mZ = 3;
    float mParticleRadius           = static_cast<float>(0.03);
    float mKernelRadius             = static_cast<float>(0.11);
    float mKernelFactor             = static_cast<float>(1);
    float mBoundaryCollisionCoeff   = static_cast<float>(0.175);
    float mRestDensity              = static_cast<float>(6378.0);
    float mMass                     = static_cast<float>(1);
    float mTimeStep                 = static_cast<float>(0.0083);
    glm::vec3 mGravity              = glm::vec3(0, -10.0, 0.0);
    float mRelaxationEpsilon        = static_cast<float>(600);
    float mSCorrDeltaQ              = static_cast<float>(0.032); //was 0.3 * 0.1
    float mSCorrK                   = static_cast<float>(0.002);
    float mSCorrN                   = static_cast<float>(4);//1.57
    float mCXsph                    = static_cast<float>(0.01);
    float mEpsilonVorticity         = static_cast<float>(0.0);
    int mSubsteps                   = 2;

    std::vector<IParticlesData*> mParticlesData;
    PBDSolverParameters(): mParticleDiameter(2.0f * mParticleRadius) {
        //mKernelRadius = 1.5f * mParticleRadius;
    }
    float getDiameter() { return mParticleDiameter; }
private:
    friend class PBDPackedSolver;
    float mParticleDiameter = static_cast<float>(0);
    
};

/*
class PBDSolverParameters {
public:
    int mX = 6, mY = 10, mZ = 6;
    float mParticleRadius           = static_cast<float>(0.1);
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
    PBDSolverParameters(): mParticleDiameter(2.0f * mParticleRadius) {
        //mKernelRadius = 1.5f * mParticleRadius;
    }
    float getDiameter() { return mParticleDiameter; }
private:
    friend class PBDPackedSolver;
    float mParticleDiameter = static_cast<float>(0);
    
};
*/

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