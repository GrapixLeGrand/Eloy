#pragma once

#include "PBDSolver.hpp"

namespace Eloy {

class PBDSolverParameters;
class PBDPackedSolver : public PBDSolver {
private:

    std::vector<glm::vec3> mPositionsStar;
    std::vector<float> mDensities;
    std::vector<glm::vec3> mAngularVelocities;
    std::vector<glm::vec3> mParallelViscosities;

    //std::vector<std::vector<int>> mNeighbors;
    std::vector<float> mLambdas;
    std::vector<glm::vec3> mPressures;

    std::vector<std::vector<int>> mUniformGrid;
    int mGridX = 0, mGridY = 0, mGridZ = 0; //sizes of the grid
    float mCellSize = static_cast<float>(0); //size of side length of a single grid cell
    int mNumGridCells = 0; //total amount of grid cells

    void findNeighbors();
    inline int get_cell_id(glm::vec3 position);
    inline float inside_kernel(float distance);
    inline float inside_kernel_2(float distance2);
    inline float s_coor(float rl);
    
    double mSolverMs = 0.0;
    double mNeighborMs = 0.0;
    double mSolverFullMs = 0.0;

public:

PBDPackedSolver(const PBDSolverParameters& parameters);
virtual void step();
virtual bool imgui();

};
}