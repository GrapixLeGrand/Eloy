#pragma once


#include "glm/glm.hpp"
#include "Kernels.hpp"
#include "AABB.hpp"

#include <vector>
#include <unordered_set>
#include <string>

namespace Eloy {

class PBDSolverParameters;

class PBDVerletSolver {
public:

    enum SolverMode {
       BASIC_SINGLE_CORE,
       BASIC_MULTI_CORE 
    };

    enum NeighborMode {
       VERLET_BASIC,
       VERLET_MINIMAL 
    };

private:

    int mX = 0, mY = 0, mZ = 0;
    AABB mAABB;

    float mParticleRadius;
    float mParticleDiameter;
    float mKernelRadius;
    float mkernelFactor;
    float mBoundaryCollisionCoeff;

    CubicKernel mCubicKernel;

    //rest density of fluid
    float mRestDensity;
    //mass of each particle
    float mMass;
    glm::vec3 mGravity;
    
    float mRelaxationEpsilon;
    float mSCorrDeltaQ;
    float mSCorrK;
    float mSCorrN;

    float mCXsph;
    float mEpsilonVorticity;

    float mEpsilonCollision = static_cast<float>(0.01);

    float mTimeStep;
    int mSubsteps = 1;

    std::vector<glm::vec3> mVelocities;
    std::vector<glm::vec3> mPositions;
    std::vector<glm::vec3> mPositionsStar;
    std::vector<float> mDensities;
    std::vector<glm::vec3> mAngularVelocities;
    std::vector<glm::vec3> mParallelViscosities;

    std::vector<std::vector<int>> mNeighbors;
    std::vector<float> mLambdas;
    std::vector<glm::vec3> mPressures;

    std::vector<std::vector<int>> mUniformGrid;
    int mGridX = 0, mGridY = 0, mGridZ = 0; //sizes of the grid
    float mCellSize = static_cast<float>(0); //size of side length of a single grid cell
    int mNumGridCells = 0; //total amount of grid cells

    std::unordered_set<int> mCellsUsedSet;
    std::vector<int> mCellsUsedStorage;
    std::vector<std::vector<int>> mCellsPrecomputedNeighbors;

    int mNumParticles = 0;

    //utilitary functions
    inline float s_coor(float rl);
    inline float resolve_collision(float value, float min, float max);
    
    void findNeighborsUniformGrid();
    void findNeighborsUniformGridMinimalStrategy();

    inline void clearNeighbors();
    inline int get_cell_id(glm::vec3 position);
    inline bool check_index(int i, int min, int max);
    void resize(size_t newSize);
    std::vector<glm::vec4> mColors;

    //for profiling
    unsigned long long mSolverCycles = 0;
    unsigned long long mNeighborCycles = 0;

    double mSolverMs = 0.0;
    double mNeighborMs = 0.0;
    double mSolverFullMs = 0.0;
    
    void stepBasisSingleThreaded();
    void stepBasisMultiThreaded();
    void findNeighbors();

    SolverMode mSolverMode = BASIC_SINGLE_CORE;
    NeighborMode mNeighborMode = VERLET_BASIC;
public:



friend class IParticlesData;
friend class AABBParticlesData;
friend class PBDVerletSolverImGui;

PBDVerletSolver(const PBDSolverParameters& parameters);
void step();


const std::vector<glm::vec3>& getPositions() const;
const std::vector<glm::vec4>& getColors() const;

float getDiameter() {
    return mParticleDiameter;
}

//todo
void getParameters(PBDSolverParameters& out) const;
void writeParticlesToJson(const std::string& filepath);

};

}