#pragma once

#include <vector>
#include "glm/glm.hpp"
#include "Kernels.hpp"
#include "AABB.hpp"


namespace Eloy {

class EngineParameters;

class Engine {
private:

    int mX = 0, mY = 0, mZ = 0;
    AABB mAABB;

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
    void resize(size_t newSize);

    std::vector<glm::vec4> mColors;
public:
friend class IParticlesData;
friend class AABBParticlesData;

Engine(const EngineParameters& parameters);
void step(float dt);

const std::vector<glm::vec3>& getPositions() const;
const std::vector<glm::vec4>& getColors() const;


};

}