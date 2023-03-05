#pragma once

#include "glm/glm.hpp"
#include "Kernels.hpp"
#include "AABB.hpp"
#include "PBDSolverParameters.hpp"

#include "imgui.h"
#include "imGuIZMO.quat/imGuIZMOquat.h"

#include <string>
#include <functional>

namespace Eloy {



class PBDSolver {
private:
protected:
    
    PBDSolverParameters mParametersSave;
    PBDSolverParameters mParameters;
    CubicKernel mCubicKernel;
    AABB mAABB;
    std::vector<glm::vec3> mVelocities;
    std::vector<glm::vec3> mPositions;
    std::vector<glm::vec4> mColors;
    
    //InjectionFunction mPostStepFun;

    int mNumParticles = 0;

    void resize(size_t newSize) {
        mPositions.resize(newSize);
        mVelocities.resize(newSize);
        mColors.resize(newSize);
    }

    void writeParticlesToJson(const std::string& filepath);

public:

    friend class IParticlesData;
    friend class AABBParticlesData;
    PBDSolver(const PBDSolverParameters& params);

virtual void step() = 0;
virtual bool imgui();
virtual void reset();


const std::vector<glm::vec3>& getPositions() const;
const std::vector<glm::vec4>& getColors() const;
const PBDSolverParameters& getParameters() const { return mParameters; }

//void setBeforeStepFunction(InjectionFunction newFun) { mPostStepFun = newFun; }


};

};