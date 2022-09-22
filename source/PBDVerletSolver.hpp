#pragma once


#include "glm/glm.hpp"
#include "Kernels.hpp"
#include "AABB.hpp"

#include <vector>
#include <unordered_set>
#include <set>
#include <string>
#include "PBDSolver.hpp"

namespace Eloy {

class PBDSolverParameters;

class PBDVerletSolver : public PBDSolver {
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

    //PBDSolverParameters mParameters;

    //int mX = 0, mY = 0, mZ = 0;
    //AABB mAABB;
    int selectedSolver = 1;
    int selectedNeighbor = 0;

    
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

    //int mNumParticles = 0;

    //utilitary functions
    inline float s_coor(float rl);
    inline float resolve_collision(float value, float min, float max);
    
    void findNeighborsUniformGrid();
    void findNeighborsUniformGridMinimalStrategy();

    inline void clearNeighbors();
    inline int get_cell_id(glm::vec3 position);
    inline bool check_index(int i, int min, int max);
    //void resize(size_t newSize);
    

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


friend class PBDSolverImGui;

PBDVerletSolver(const PBDSolverParameters& parameters);
virtual void step();
virtual bool imgui();

const std::vector<glm::vec3>& getPositions() const;
const std::vector<glm::vec4>& getColors() const;

float getDiameter() {
    return 2.0f * mParameters.mParticleRadius;
}

//todo
PBDSolverParameters getParameters() const;
void writeParticlesToJson(const std::string& filepath);

};

}