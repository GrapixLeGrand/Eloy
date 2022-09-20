#include "Eloy.hpp"

#include "glm/gtc/constants.hpp"
#include "profiling/tsc_x86.hpp"
#include "nlohmann/json.hpp"
#include "omp.h"

#include <algorithm>
#include <chrono>

namespace Eloy {    

PBDVerletSolver::PBDVerletSolver(const PBDSolverParameters& parameters): PBDSolver(parameters) {

    /*mX = parameters.mX;
    mY = parameters.mY;
    mZ = parameters.mZ;*/

    mAABB = AABB({0, 0, 0} , {mParameters.mX, mParameters.mY, mParameters.mZ});

    /*mParticleRadius = parameters.mParticuleRadius;
    mParticleDiameter = parameters.mParticuleRadius * static_cast<float>(2);
    mKernelRadius = parameters.mKernelRadius;
    mkernelFactor = parameters.mKernelFactor;
    mBoundaryCollisionCoeff = parameters.mBoundaryCollisionCoeff;*/

    //mCubicKernel = CubicKernel(mParameters.mKernelRadius, parameters.mKernelFactor);

    /*mRestDensity = parameters.mRestDensity;
    mMass = parameters.mMass;
    mGravity = parameters.mGravity;

    mRelaxationEpsilon = parameters.mRelaxationEpsilon;
    mSCorrDeltaQ = parameters.mSCorrDeltaQ;
    mSCorrK = parameters.mSCorrK;
    mSCorrN = parameters.mSCorrN;

    mCXsph = parameters.mCXsph;
    mEpsilonVorticity = parameters.mEpsilonVorticity;

    mTimeStep = parameters.mTimeStep;
    mSubsteps = parameters.mSubsteps;*/

    for (const IParticlesData* data : parameters.mParticlesData) {
        data->addParticlesData(this);
    }
    
    mPositionsStar = std::vector<glm::vec3>(mNumParticles, glm::vec3(0));
    mLambdas = std::vector<float>(mNumParticles, static_cast<float>(0));
    mPressures = std::vector<glm::vec3>(mNumParticles, glm::vec3(0));
    mDensities = std::vector<float>(mNumParticles, static_cast<float>(0));
    mAngularVelocities = std::vector<glm::vec3>(mNumParticles, glm::vec3(0));
    mParallelViscosities = std::vector<glm::vec3>(mNumParticles, glm::vec3(0));

    mNeighbors = std::vector<std::vector<int>>(mNumParticles, std::vector<int>{});
    mCellSize = mParameters.mKernelRadius * 0.5f;

    mGridX = static_cast<float>(mParameters.mX / mCellSize) + 1;
    mGridY = static_cast<float>(mParameters.mY / mCellSize) + 1;
    mGridZ = static_cast<float>(mParameters.mZ / mCellSize) + 1;

    mNumGridCells = mGridX * mGridY * mGridZ;
    mUniformGrid = std::vector<std::vector<int>>(mNumGridCells, std::vector<int>{});

    {//minimal strategy
        mCellsUsedStorage.reserve(mNumGridCells);
        mCellsPrecomputedNeighbors = std::vector<std::vector<int>>(mNumGridCells, std::vector<int>{});
        for (int y = 0; y < mGridY; y++) {
            for (int x = 0; x < mGridX; x++) {
                for (int z = 0; z < mGridZ; z++) {
                    
                    int index = y * mGridX * mGridZ + x * mGridZ + z;

                    int yLower = y - 1; int yUpper = y + 1;
                    int xLower = x - 1; int xUpper = x + 1;
                    int zLower = z - 1; int zUpper = z + 1;

                    yLower = (yLower >= 0) ? yLower : 0; 
                    xLower = (xLower >= 0) ? xLower : 0;
                    zLower = (zLower >= 0) ? zLower : 0;

                    yUpper = (yUpper >= mGridY) ? mGridY - 1 : yUpper;
                    xUpper = (xUpper >= mGridX) ? mGridX - 1 : xUpper;
                    zUpper = (zUpper >= mGridZ) ? mGridZ - 1 : zUpper;

                    for (int yy = yLower; yy <= yUpper; yy++) {
                        for (int xx = xLower; xx <= xUpper; xx++) {
                            for (int zz = zLower; zz <= zUpper; zz++) {
                                int indexNeighbor = yy * mGridX * mGridZ + xx * mGridZ + zz;
                                mCellsPrecomputedNeighbors[index].push_back(indexNeighbor);
                            }
                        }
                    }


                }
            }
        }

    }

}

#define CHECK_NAN_VEC(V) \
    assert(!glm::isnan(V.x) || !glm::isnan(V.y) || !glm::isnan(V.z) && "the vector has nan comp"); \
    assert(!glm::isinf(V.x) || !glm::isinf(V.y) || !glm::isinf(V.z) && "the vector has inf comp");
    

#define CHECK_NAN_VAL(V) \
    assert(!glm::isnan(V) && "the value is nan"); \
    assert(!glm::isinf(V) && "the value is inf");


inline float PBDVerletSolver::s_coor(float rl) {
    float result = static_cast<float>(0);
    float W = mCubicKernel.W(mParameters.mSCorrDeltaQ);
    if (W > 0.001f) { //glm::epsilon<float>()
        result = -mParameters.mSCorrK * std::pow(mCubicKernel.W(rl) / W, mParameters.mSCorrN);
    }
    CHECK_NAN_VAL(result);
    return result;
}

/*
inline float PBDVerletSolver::resolve_collision(float value, float min, float max) {

    if (value <= min) {
        return mParameters.mEpsilonCollision;
    }

    if (value > max) {
        return max - mParameters.mEpsilonCollision;
    }

    return value;
}*/


//Taken from Lustrine on github
inline glm::vec3 solve_boundary_collision_constraint(glm::vec3 n, glm::vec3 p0, glm::vec3 p, float d) {
    float C = glm::dot(n, (p - p0)) - d;
    if (C >= 0) {
        return glm::vec3(0.0);
    }

    // https://matthias-research.github.io/pages/publications/posBasedDyn.pdf Eq(9)
    glm::vec3 dC = n;
    float s = C / glm::dot(dC, dC);
    glm::vec3 dp = -s * dC;

    return dp;
}

void PBDVerletSolver::findNeighbors() {
    auto startNeigbors = std::chrono::steady_clock::now();
    switch (mNeighborMode) {
        case VERLET_BASIC:
            this->findNeighborsUniformGrid();
        break;
        case VERLET_MINIMAL:
            this->findNeighborsUniformGridMinimalStrategy();
        break;
        default:
        assert(false);
    }
    auto endNeigbors = std::chrono::steady_clock::now();
    mNeighborMs = std::chrono::duration<double, std::milli> (endNeigbors - startNeigbors).count();
}

void PBDVerletSolver::step() {

    auto startAll = std::chrono::steady_clock::now();

    switch (mSolverMode) {
        case BASIC_SINGLE_CORE:
            this->stepBasisSingleThreaded();
        break;
        case BASIC_MULTI_CORE:
            this->stepBasisMultiThreaded();
        break;
        default:
        assert(false);
    }

    auto endAll = std::chrono::steady_clock::now();
    mSolverFullMs = std::chrono::duration<double, std::milli> (endAll - startAll).count();
    mSolverMs = mSolverFullMs - mNeighborMs;
}

void PBDVerletSolver::stepBasisMultiThreaded() {

    //integration
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] += mParameters.mGravity * mParameters.mMass * mParameters.mTimeStep;
        CHECK_NAN_VEC(mVelocities[i]);
        mPositionsStar[i] = mPositions[i] + mVelocities[i] * mParameters.mTimeStep; //prediction
        CHECK_NAN_VEC(mPositionsStar[i]);
    }

    findNeighbors();

    for (int s = 0; s < mParameters.mSubsteps; s++) {

        //solve pressure
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < mNumParticles; i++) {

            float densitiy = 0.0;
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                float len = glm::length(ij);
                densitiy += mParameters.mMass * mCubicKernel.W(len);
                CHECK_NAN_VEC(ij);
            }
            densitiy += mParameters.mMass * mCubicKernel.W(0.0f);

            //equation 1
            float constraint_i = (densitiy / mParameters.mRestDensity) - static_cast<float>(1);
            float constraint_gradient_sum = static_cast<float>(0);
            glm::vec3 grad_current_p = glm::vec3(0.0);

            CHECK_NAN_VAL(constraint_i);
            
            //equation 8
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 temp = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                glm::vec3 neighbor_grad = -(mParameters.mMass / mParameters.mRestDensity) * mCubicKernel.WGrad(temp);
                CHECK_NAN_VEC(neighbor_grad);
                constraint_gradient_sum += glm::dot(neighbor_grad, neighbor_grad);
                grad_current_p -= neighbor_grad;
            }

            CHECK_NAN_VEC(grad_current_p);
            constraint_gradient_sum += glm::dot(grad_current_p, grad_current_p);

            mLambdas[i] = static_cast<float>(0);
            if (constraint_gradient_sum > 0.0f) {
                mLambdas[i] = -constraint_i / (constraint_gradient_sum + mParameters.mRelaxationEpsilon);
                CHECK_NAN_VAL(mLambdas[i]);
            }

        }

        #pragma omp parallel for schedule(static)
        for (int i = 0; i < mNumParticles; i++) {

            //equation 13 (applying pressure force correction)
            mPressures[i] = glm::vec3(0);
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                CHECK_NAN_VEC(ij);
                mPressures[i] += (mLambdas[i] + mLambdas[j] + s_coor(glm::length(ij))) * mCubicKernel.WGrad(ij); //
                CHECK_NAN_VEC(mPressures[i]);
            }

            mPressures[i] *= (1.0f / (mParameters.mRestDensity * mParameters.mMass));
            
        }

        #pragma omp parallel for schedule(static)
        for (int i = 0; i < mNumParticles; i++) {
            mPositionsStar[i] += mPressures[i];
            glm::vec3 dp = glm::vec3(0.0);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(1, 0, 0), glm::vec3(0, 0, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(-1, 0, 0), glm::vec3(mParameters.mX, 0, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 1, 0), glm::vec3(0, 0, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, -1, 0), glm::vec3(0, mParameters.mY, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 0, 1), glm::vec3(0, 0, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 0, -1), glm::vec3(0, 0, mParameters.mZ), mPositionsStar[i], mParameters.mParticleRadius);
            mPositionsStar[i] += dp;
        }

    }


    //recompute the density or take the one from the initial guess?
    //we do the density and the vorticity in the same loop to better utilize the cache
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mNumParticles; i++) {
        mDensities[i] = 0.0f;
        
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            //density
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            mDensities[i] += mParameters.mMass * mCubicKernel.W(glm::length(ij));
        }
        mDensities[i] += mParameters.mMass * mCubicKernel.W(0.0f);

        mAngularVelocities[i] = {0, 0, 0};
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            //angular velocity
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            glm::vec3 vij = mVelocities[mNeighbors[i][j]] - mVelocities[i];
            mAngularVelocities[i] += glm::cross(vij, mCubicKernel.WGrad(ij)) * (mParameters.mMass / mDensities[i]);
        }
    }

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] = (mPositionsStar[i] - mPositions[i]) / mParameters.mTimeStep;
        
        //vorticity confinment
        glm::vec3 N = {0, 0, 0};
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            N += mCubicKernel.WGrad(ij) * (mParameters.mMass / mDensities[i]) * glm::length(mAngularVelocities[mNeighbors[i][j]]);
        }
        float NLength = glm::length(N);
        if (NLength > 0.0f) {
            N /= NLength;
            glm::vec3 vorticity = glm::cross(N, mAngularVelocities[i]) * mParameters.mEpsilonVorticity;
            mVelocities[i] += mParameters.mTimeStep * mParameters.mMass * vorticity; 
        }
    }

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mNumParticles; i++) {
        //xsph viscosity
        mParallelViscosities[i] = {0, 0, 0};
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            glm::vec3 vij = mVelocities[mNeighbors[i][j]] - mVelocities[i];
            if (mDensities[mNeighbors[i][j]] > 0.0f)
                mParallelViscosities[i] += vij * (mParameters.mMass / mDensities[mNeighbors[i][j]]) * mCubicKernel.W(glm::length(ij));
        }
    }

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] += mParallelViscosities[i] * mParameters.mCXsph;
        mPositions[i] = mPositionsStar[i];
    }

}

void PBDVerletSolver::stepBasisSingleThreaded() {

    //integration
    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] += mParameters.mGravity * mParameters.mMass * mParameters.mTimeStep;
        CHECK_NAN_VEC(mVelocities[i]);
        mPositionsStar[i] = mPositions[i] + mVelocities[i] * mParameters.mTimeStep; //prediction
        CHECK_NAN_VEC(mPositionsStar[i]);
    }

    mNeighborCycles = start_tsc();
    findNeighbors();
    mNeighborCycles = stop_tsc(mNeighborCycles);

    mSolverCycles = start_tsc();

    for (int s = 0; s < mParameters.mSubsteps; s++) {

        //solve pressure
        for (int i = 0; i < mNumParticles; i++) {

            float densitiy = 0.0;
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                float len = glm::length(ij);
                densitiy += mParameters.mMass * mCubicKernel.W(len);
                CHECK_NAN_VEC(ij);
            }
            densitiy += mParameters.mMass * mCubicKernel.W(0.0f);

            //equation 1
            float constraint_i = (densitiy / mParameters.mRestDensity) - static_cast<float>(1);
            float constraint_gradient_sum = static_cast<float>(0);
            glm::vec3 grad_current_p = glm::vec3(0.0);

            CHECK_NAN_VAL(constraint_i);
            
            //equation 8
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 temp = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                glm::vec3 neighbor_grad = -(mParameters.mMass / mParameters.mRestDensity) * mCubicKernel.WGrad(temp);
                CHECK_NAN_VEC(neighbor_grad);
                constraint_gradient_sum += glm::dot(neighbor_grad, neighbor_grad);
                grad_current_p -= neighbor_grad;
            }

            CHECK_NAN_VEC(grad_current_p);
            constraint_gradient_sum += glm::dot(grad_current_p, grad_current_p);

            mLambdas[i] = static_cast<float>(0);
            if (constraint_gradient_sum > 0.0f) {
                mLambdas[i] = -constraint_i / (constraint_gradient_sum + mParameters.mRelaxationEpsilon);
                CHECK_NAN_VAL(mLambdas[i]);
            }

        }

        for (int i = 0; i < mNumParticles; i++) {

            //equation 13 (applying pressure force correction)
            mPressures[i] = glm::vec3(0);
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                CHECK_NAN_VEC(ij);
                mPressures[i] += (mLambdas[i] + mLambdas[j] + s_coor(glm::length(ij))) * mCubicKernel.WGrad(ij); //
                CHECK_NAN_VEC(mPressures[i]);
            }

            mPressures[i] *= (1.0f / (mParameters.mRestDensity * mParameters.mMass));
            
        }

        for (int i = 0; i < mNumParticles; i++) {
            mPositionsStar[i] += mPressures[i];
            glm::vec3 dp = glm::vec3(0.0);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(1, 0, 0), glm::vec3(0, 0, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(-1, 0, 0), glm::vec3(mParameters.mX, 0, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 1, 0), glm::vec3(0, 0, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, -1, 0), glm::vec3(0, mParameters.mY, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 0, 1), glm::vec3(0, 0, 0), mPositionsStar[i], mParameters.mParticleRadius);
            dp += mParameters.mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 0, -1), glm::vec3(0, 0, mParameters.mZ), mPositionsStar[i], mParameters.mParticleRadius);
            mPositionsStar[i] += dp;
        }

    }


    //recompute the density or take the one from the initial guess?
    //we do the density and the vorticity in the same loop to better utilize the cache
    for (int i = 0; i < mNumParticles; i++) {
        mDensities[i] = 0.0f;
        
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            //density
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            mDensities[i] += mParameters.mMass * mCubicKernel.W(glm::length(ij));
        }
        mDensities[i] += mParameters.mMass * mCubicKernel.W(0.0f);

        mAngularVelocities[i] = {0, 0, 0};
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            //angular velocity
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            glm::vec3 vij = mVelocities[mNeighbors[i][j]] - mVelocities[i];
            mAngularVelocities[i] += glm::cross(vij, mCubicKernel.WGrad(ij)) * (mParameters.mMass / mDensities[i]);
        }
    }

    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] = (mPositionsStar[i] - mPositions[i]) / mParameters.mTimeStep;

        //vorticity confinment
        glm::vec3 N = {0, 0, 0};
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            N += mCubicKernel.WGrad(ij) * (mParameters.mMass / mDensities[i]) * glm::length(mAngularVelocities[mNeighbors[i][j]]);
        }
        float NLength = glm::length(N);
        if (NLength > 0.0f) {
            N /= NLength;
            glm::vec3 vorticity = glm::cross(N, mAngularVelocities[i]) * mParameters.mEpsilonVorticity;
            mVelocities[i] += mParameters.mTimeStep * mParameters.mMass * vorticity; 
        }

        //xsph viscosity
        glm::vec3 viscosity = {0, 0, 0};
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            glm::vec3 vij = mVelocities[mNeighbors[i][j]] - mVelocities[i];
            if (mDensities[mNeighbors[i][j]] > 0.0f)
                viscosity += vij * (mParameters.mMass / mDensities[mNeighbors[i][j]]) * mCubicKernel.W(glm::length(ij));
        }
        mVelocities[i] += viscosity * mParameters.mCXsph;
        mPositions[i] = mPositionsStar[i];
    }

    mSolverCycles = stop_tsc(mSolverCycles);
    
}

void PBDVerletSolver::resize(size_t newSize) {
    mPositions.resize(newSize);
    mVelocities.resize(newSize);
    mColors.resize(newSize);
}

inline int PBDVerletSolver::get_cell_id(glm::vec3 position) {
    //position = glm::clamp(position, glm::vec3(simulation->mCellSize * 0.5f), glm::vec3(simulation->domainX - simulation->mCellSize * 0.5f, simulation->domainY - simulation->mCellSize * 0.5f, simulation->domainZ - simulation->mCellSize * 0.5f));
    CHECK_NAN_VEC(position);
    position /= mCellSize;
    int cell_id =
            (static_cast<int>(position.y)) * mGridX * mGridZ +
            (static_cast<int>(position.x)) * mGridZ +
            (static_cast<int>(position.z));
    cell_id = glm::clamp(cell_id, 0, mNumGridCells - 1);
    return cell_id;
}

inline bool PBDVerletSolver::check_index(int i, int min, int max) {
    return (i >= min && i < max);
}

void PBDVerletSolver::findNeighborsUniformGrid() {

    for (int i = 0; i < mNumParticles; i++) {
        mNeighbors[i].clear();
    }

    for (int i = 0; i < mNumGridCells; i++) {
        mUniformGrid[i].clear();
    }

    for (int i = 0; i < mNumParticles; i++) {
        int cell_id = get_cell_id(mPositionsStar[i]);
        //glm::vec3& position = mPositionsStar[i];
        assert(cell_id >= 0 && cell_id < mNumGridCells && "index must be in range");
        //if (cell_id >= 0 && cell_id < mNumGridCells)
            mUniformGrid[cell_id].push_back(i);
    }

    for (int yy = 0; yy < mGridY; yy++) {
        for (int xx = 0; xx < mGridX; xx++) {
            for (int zz = 0; zz < mGridZ; zz++) {

                int cell_id = 
                    yy * mGridX * mGridZ +
                    xx * mGridZ + 
                    zz;

                if (mUniformGrid[cell_id].empty() == true) {
                    continue;
                }

                int yLower = yy - 1; int yUpper = yy + 1;
                int xLower = xx - 1; int xUpper = xx + 1;
                int zLower = zz - 1; int zUpper = zz + 1;

                yLower = (yLower >= 0) ? yLower : 0; 
                xLower = (xLower >= 0) ? xLower : 0;
                zLower = (zLower >= 0) ? zLower : 0;

                yUpper = (yUpper >= mGridY) ? mGridY - 1 : yUpper;
                xUpper = (xUpper >= mGridX) ? mGridX - 1 : xUpper;
                zUpper = (zUpper >= mGridZ) ? mGridZ - 1 : zUpper;

                for (int y = yLower; y <= yUpper; y++) {
                    for (int x = xLower; x <= xUpper; x++) {
                        for (int z = zLower; z <= zUpper; z++) {

                            int neighbor_cell_id =
                                y * mGridX * mGridZ +
                                x * mGridZ + 
                                z;

                            if (mUniformGrid[neighbor_cell_id].empty() == true) {
                                continue;
                            }

                            for (int i = 0; i < mUniformGrid[cell_id].size(); i++) {
                                const int current_index = mUniformGrid[cell_id][i];
                                /*if (current_index >= ptr_solid_start) {
                                    continue;
                                }*/
                                const glm::vec3& self = mPositionsStar[current_index];
                                for (int j = 0; j < mUniformGrid[neighbor_cell_id].size(); j++) {
                                    const int neighbor_index = mUniformGrid[neighbor_cell_id][j];
                                    const glm::vec3& other = mPositionsStar[neighbor_index];
                                    const glm::vec3 tmp = self - other;
                                    if (glm::dot(tmp, tmp) <= mParameters.mKernelRadius * mParameters.mKernelRadius) {
                                        mNeighbors[current_index].push_back(neighbor_index);
                                    }
                                }
                            } //end distance check

                        }
                    }
                } //end neighbor cells checking 


            }
        }
    } //end grid checking

}


void PBDVerletSolver::findNeighborsUniformGridMinimalStrategy() {

    mCellsUsedSet.clear();
    mCellsUsedStorage.clear();

    for (int i = 0; i < mNumParticles; i++) {
        mNeighbors[i].clear();
    }

    for (int i = 0; i < mNumGridCells; i++) {
        mUniformGrid[i].clear();
    }

    for (int i = 0; i < mNumParticles; i++) {
        int cell_id = get_cell_id(mPositionsStar[i]);
        mUniformGrid[cell_id].push_back(i);
        if (mCellsUsedSet.find(cell_id) == mCellsUsedSet.end()) {
            mCellsUsedSet.insert(cell_id);
            mCellsUsedStorage.push_back(cell_id);
        }
    }

    for (int cellId : mCellsUsedStorage) {
        for (int neighborCellId : mCellsPrecomputedNeighbors[cellId]) {
            
            if (mUniformGrid[neighborCellId].empty() == true) {
                continue;
            }

            for (int i = 0; i < mUniformGrid[cellId].size(); i++) {
                const int current_index = mUniformGrid[cellId][i];
                const glm::vec3& self = mPositionsStar[current_index];
                for (int j = 0; j < mUniformGrid[neighborCellId].size(); j++) {
                    const int neighbor_index = mUniformGrid[neighborCellId][j];
                    const glm::vec3& other = mPositionsStar[neighbor_index];
                    const glm::vec3 tmp = self - other;
                    if (glm::dot(tmp, tmp) <= mParameters.mKernelRadius * mParameters.mKernelRadius) {
                        mNeighbors[current_index].push_back(neighbor_index);
                    }
                }
            } //end distance check

        }
    }
}

PBDSolverParameters PBDVerletSolver::getParameters() const {
    return mParameters;
}
/*
void PBDVerletSolver::getParameters(PBDSolverParameters& out) const {

    out.mParticuleRadius = mParticleRadius;
    out.mCXsph = mCXsph;
    out.mEpsilonVorticity = mEpsilonVorticity;
    out.mGravity = mGravity;
    out.mKernelFactor = mkernelFactor;
    out.mMass = mMass;
    out.mRelaxationEpsilon = mRelaxationEpsilon;
    out.mRestDensity = mRestDensity;
    out.mSCorrDeltaQ = mSCorrDeltaQ;
    out.mSCorrK = mSCorrK;
    out.mSCorrN = mSCorrN;
    out.mTimeStep = mTimeStep;
    out.mX = mX;
    out.mY = mY;
    out.mZ = mZ;
    out.mBoundaryCollisionCoeff = mBoundaryCollisionCoeff;
    out.mSubsteps = mSubsteps;
    out.mTimeStep = mTimeStep;
    out.mKernelRadius = mKernelRadius;
}*/

const std::vector<glm::vec3>& PBDVerletSolver::getPositions() const {
    return mPositions;
}

const std::vector<glm::vec4>& PBDVerletSolver::getColors() const {
    return mColors;
}


void PBDVerletSolver::writeParticlesToJson(const std::string& filepath) {
    auto jsonParticlesPositions = nlohmann::json::array();

    for (int i = 0; i < mNumParticles; i++) {
        auto jsonParticle = nlohmann::json::array();
        for (int j = 0; j < 3; j++) {
            jsonParticle.push_back(mPositions[i][j]);
        }
        jsonParticlesPositions .push_back(jsonParticle);
    }

    auto jsonParticlesColors = nlohmann::json::array();
    for (int i = 0; i < mNumParticles; i++) {
        auto jsonParticle = nlohmann::json::array();
        for (int j = 0; j < 4; j++) {
            jsonParticle.push_back(mColors[i][j]);
        }
        jsonParticlesColors.push_back(jsonParticle);
    }
    
    auto jsonParticlesData = nlohmann::json::object();
    jsonParticlesData["positions"] = jsonParticlesPositions;
    jsonParticlesData["colors"] = jsonParticlesColors;

    std::ofstream file(filepath);
    file << jsonParticlesData;
}


bool PBDVerletSolver::imgui() {
    bool quit = PBDSolver::imgui();
    if (ImGui::BeginTabItem("Parameters")) {
        ImGui::Text("%d particles %lf ms", mNumParticles, mSolverMs);
        ImGui::Text("%d cells %lf ms", mNumGridCells, mNeighborMs);
        ImGui::Text("simulation (everything) %lf ms (%lf fps)", mSolverFullMs, (1.0 / mSolverFullMs) * 1000.0);

        static const char* solverTypes[]{"single-thread", "multi-threaded"}; 
        ImGui::Combo("solver", &selectedSolver, solverTypes, IM_ARRAYSIZE(solverTypes));
        mSolverMode = (PBDVerletSolver::SolverMode) selectedSolver;

        static const char* neighborTypes[]{"basic-verlet", "minimalist"}; 
        ImGui::Combo("neighbors", &selectedNeighbor, neighborTypes, IM_ARRAYSIZE(neighborTypes));
        mNeighborMode = (PBDVerletSolver::NeighborMode) selectedNeighbor;
        if (ImGui::Button("save state")) {
            writeParticlesToJson(ELOY_BUILD_DIRECTORY"/save.json");
        }
    }
    return quit;
}


}
