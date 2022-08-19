#pragma once

#include <vector>
#include "glm/glm.hpp"
#include "Kernels.hpp"
#include "EngineParameters.hpp"

namespace Eloy {

class Engine {
private:

    int mX = 0, mY = 0, mZ = 0;
    float mParticleRadius;
    float mParticleDiameter;
    float mKernelRadius;
    float mkernelFactor;

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

    std::vector<glm::vec3> mVelocities;
    std::vector<glm::vec3> mPositions;
    std::vector<glm::vec3> mPositionsStar;
    std::vector<std::vector<int>> mNeighbors;
    std::vector<float> mLambdas;

    std::vector<std::vector<int>> mUniformGrid;
    int mGridX = 0, mGridY = 0, mGridZ = 0; //sizes of the grid
    float mCellSize = static_cast<float>(0); //size of side length of a single grid cell
    int mNumGridCells = 0; //total amount of grid cells

    int mNumParticles = 0;

    //utilitary functions
    inline float s_coor(float rl);
    inline float resolve_collision(float value, float min, float max);
    void findNeighborsUniformGrid();
    inline void clearNeighbors();
    inline int get_cell_id(glm::vec3 position);
    inline bool check_index(int i, int min, int max);

public:

Engine(const EngineParameters& parameters);
void step(float dt);


};

}