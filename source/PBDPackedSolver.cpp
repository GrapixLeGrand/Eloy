#include "PBDPackedSolver.hpp"

#include <algorithm>
#include <chrono>

namespace Eloy {

PBDPackedSolver::PBDPackedSolver(const PBDSolverParameters& parameters): PBDSolver(parameters) {

    mPositionsStar = std::vector<glm::vec3>(mNumParticles, glm::vec3(0));
    mLambdas = std::vector<float>(mNumParticles, static_cast<float>(0));
    mPressures = std::vector<glm::vec3>(mNumParticles, glm::vec3(0));
    mDensities = std::vector<float>(mNumParticles, static_cast<float>(0));
    mAngularVelocities = std::vector<glm::vec3>(mNumParticles, glm::vec3(0));
    mParallelViscosities = std::vector<glm::vec3>(mNumParticles, glm::vec3(0));

    mCellSize = mParameters.mKernelRadius * 0.5f;

    mGridX = static_cast<int>(std::ceil(mParameters.mX / mCellSize)) + 2;
    mGridY = static_cast<int>(std::ceil(mParameters.mY / mCellSize)) + 2;
    mGridZ = static_cast<int>(std::ceil(mParameters.mZ / mCellSize)) + 2;
    
    mNumGridCells = mGridX * mGridY * mGridZ;
    int approximateMaxNeighbors = std::pow(mCellSize / mParameters.mParticleDiameter, 3);
    printf("approx. max neighbors %d\n", approximateMaxNeighbors);
    mUniformGrid = std::vector<std::vector<int>>(mNumGridCells, std::vector<int>(approximateMaxNeighbors));
    for (auto& v : mUniformGrid) v.clear();
    
}

inline int PBDPackedSolver::get_cell_id(glm::vec3 position) {
    //position = glm::clamp(position, glm::vec3(simulation->mCellSize * 0.5f), glm::vec3(simulation->domainX - simulation->mCellSize * 0.5f, simulation->domainY - simulation->mCellSize * 0.5f, simulation->domainZ - simulation->mCellSize * 0.5f));
    //CHECK_NAN_VEC(position);
    position /= mCellSize;
    int x = glm::clamp(static_cast<int>(position.x), 0, mGridX-1) + 1;
    int y = glm::clamp(static_cast<int>(position.y), 0, mGridY-1) + 1;
    int z = glm::clamp(static_cast<int>(position.z), 0, mGridZ-1) + 1;

    int cell_id =
            y * mGridX * mGridZ +
            x * mGridZ +
            z;
    cell_id = glm::clamp(cell_id, 0, mNumGridCells - 1);
    return cell_id;
}

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

void PBDPackedSolver::findNeighbors() {
    auto startNeigbors = std::chrono::steady_clock::now();
    
    for (auto& v : mUniformGrid) v.clear();

    for (int i = 0; i < mNumParticles; i++) {
        int cell_id = get_cell_id(mPositionsStar[i]);
        assert(cell_id >= 0 && cell_id < mNumGridCells && "index must be in range");
        mUniformGrid[cell_id].push_back(i);
    }

    auto endNeigbors = std::chrono::steady_clock::now();
    mNeighborMs = std::chrono::duration<double, std::milli> (endNeigbors - startNeigbors).count();
}

inline float PBDPackedSolver::inside_kernel(float distance) {
    return (distance <= mParameters.mKernelRadius) ? 1.0 : 0.0; 
}

inline float PBDPackedSolver::inside_kernel_2(float distance2) {
    return (distance2 <= (mParameters.mKernelRadius * mParameters.mKernelRadius)) ? 1.0 : 0.0; 
}






#define CHECK_NAN_VEC(V) \
    assert(!glm::isnan(V.x) || !glm::isnan(V.y) || !glm::isnan(V.z) && "the vector has nan comp"); \
    assert(!glm::isinf(V.x) || !glm::isinf(V.y) || !glm::isinf(V.z) && "the vector has inf comp");
    

#define CHECK_NAN_VAL(V) \
    assert(!glm::isnan(V) && "the value is nan"); \
    assert(!glm::isinf(V) && "the value is inf");


inline float PBDPackedSolver::s_coor(float rl) {
    float result = static_cast<float>(0);
    float W = mCubicKernel.W(mParameters.mSCorrDeltaQ);
    if (W > 0.001f) { //glm::epsilon<float>()
        result = -mParameters.mSCorrK * std::pow(mCubicKernel.W(rl) / W, mParameters.mSCorrN);
    }
    CHECK_NAN_VAL(result);
    return result;
}

void PBDPackedSolver::step() {

    //integration
    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] += mParameters.mGravity * mParameters.mMass * mParameters.mTimeStep;
        mPositionsStar[i] = mPositions[i] + mVelocities[i] * mParameters.mTimeStep; //prediction
    }

    findNeighbors();

    for (int s = 0; s < mParameters.mSubsteps; s++) {

        for (int y = 1; y < mGridY-1; y++) {
            for (int x = 1; x < mGridX-1; x++) {
                for (int z = 1; z < mGridZ-1; z++) {
                    
                    int currentCellIndex = y * mGridX * mGridZ + x * mGridZ + z;
                    std::vector<int>& currentParticlesIndices = mUniformGrid[currentCellIndex];
                    if (currentParticlesIndices.empty()) continue;

                    for (int currentId : currentParticlesIndices) {
                        
                        float density = 0.0;
                        for (int yy = y-1; yy <= y+1; yy++) {
                            for (int xx = x-1; xx <= x+1; xx++) {
                                for (int zz = z-1; zz <= z+1; zz++) {
                                    int neighborCellIndex = yy * mGridX * mGridZ + xx * mGridZ + zz;
                                    
                                    std::vector<int>& otherParticlesIndices = mUniformGrid[neighborCellIndex];
                                    if (otherParticlesIndices.empty()) continue;
                                    
                                    for (int otherId : otherParticlesIndices) {
                                        glm::vec3 ij = mPositionsStar[currentId] - mPositionsStar[otherId];
                                        float len = glm::length(ij);
                                        density += mParameters.mMass * mCubicKernel.W(len) * inside_kernel(len);
                                    }

                                }
                            }
                        }

                        density += mParameters.mMass * mCubicKernel.W(0.0f);

                        float constraint_i = (density / mParameters.mRestDensity) - static_cast<float>(1);
                        float constraint_gradient_sum = static_cast<float>(0);
                        glm::vec3 grad_current_p = glm::vec3(0.0);


                        for (int yy = y-1; yy <= y+1; yy++) {
                            for (int xx = x-1; xx <= x+1; xx++) {
                                for (int zz = z-1; zz <= z+1; zz++) {
                                    int neighborCellIndex = yy * mGridX * mGridZ + xx * mGridZ + zz;
                                    
                                    std::vector<int>& otherParticlesIndices = mUniformGrid[neighborCellIndex];
                                    if (otherParticlesIndices.empty()) continue;
                                    
                                    for (int otherId : otherParticlesIndices) {
                                        glm::vec3 temp = mPositionsStar[currentId] - mPositionsStar[otherId];
                                        glm::vec3 neighbor_grad = -(mParameters.mMass / mParameters.mRestDensity) * mCubicKernel.WGrad(temp);
                                        CHECK_NAN_VEC(neighbor_grad);
                                        float len = glm::length(temp);
                                        constraint_gradient_sum += glm::dot(neighbor_grad, neighbor_grad) * inside_kernel(len);
                                        grad_current_p -= (neighbor_grad * inside_kernel(len));
                                    }

                                }
                            }
                        }

                        CHECK_NAN_VEC(grad_current_p);
                        constraint_gradient_sum += glm::dot(grad_current_p, grad_current_p);

                        mLambdas[currentId] = static_cast<float>(0);
                        if (constraint_gradient_sum > 0.0f) {
                            mLambdas[currentId] = -constraint_i / (constraint_gradient_sum + mParameters.mRelaxationEpsilon);
                            CHECK_NAN_VAL(mLambdas[currentId]);
                        }

                    }

                }
            }
        } //end loop for all i


        for (int y = 1; y < mGridY-1; y++) {
            for (int x = 1; x < mGridX-1; x++) {
                for (int z = 1; z < mGridZ-1; z++) {
                    
                    int currentCellIndex = y * mGridX * mGridZ + x * mGridZ + z;
                    std::vector<int>& currentParticlesIndices = mUniformGrid[currentCellIndex];
                    if (currentParticlesIndices.empty()) continue;

                    for (int currentId : currentParticlesIndices) {
                        
                        mPressures[currentId] = glm::vec3(0);
                        for (int yy = y-1; yy <= y+1; yy++) {
                            for (int xx = x-1; xx <= x+1; xx++) {
                                for (int zz = z-1; zz <= z+1; zz++) {
                                    int neighborCellIndex = yy * mGridX * mGridZ + xx * mGridZ + zz;
                                    
                                    std::vector<int>& otherParticlesIndices = mUniformGrid[neighborCellIndex];
                                    if (otherParticlesIndices.empty()) continue;
                                    
                                    for (int otherId : otherParticlesIndices) {
                                        glm::vec3 ij = mPositionsStar[currentId] - mPositionsStar[otherId];
                                        CHECK_NAN_VEC(ij);
                                        float len = glm::length(ij);
                                        glm::vec3 pressure = (mLambdas[currentId] + mLambdas[otherId] + s_coor(len)) * mCubicKernel.WGrad(ij);
                                        mPressures[currentId] += pressure * inside_kernel(len);
                                        CHECK_NAN_VEC(mPressures[currentId]);
                                    }

                                }
                            }
                        }

                        mPressures[currentId] *= (1.0f / (mParameters.mRestDensity * mParameters.mMass));
                        
                    }

                }
            }
        } //end loop for all i

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


}


}