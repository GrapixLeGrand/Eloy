#include "Eloy.hpp"

#include <algorithm>

#include "glm/gtc/constants.hpp"
#include "EngineParameters.hpp"

namespace Eloy {    

Engine::Engine(const EngineParameters& parameters) {

    mX = parameters.mX;
    mY = parameters.mY;
    mZ = parameters.mZ;

    mAABB = AABB({0, 0, 0} , {mX, mY, mZ});

    mParticleRadius = parameters.mParticuleRadius;
    mParticleDiameter = parameters.mParticuleRadius * static_cast<float>(2);
    mKernelRadius = static_cast<float>(3.1) * mParticleRadius;
    mkernelFactor = parameters.mKernelFactor;
    mBoundaryCollisionCoeff = parameters.mBoundaryCollisionCoeff;

    mCubicKernel = CubicKernel(mKernelRadius, parameters.mKernelFactor);

    mRestDensity = parameters.mRestDensity;
    mMass = parameters.mMass;
    mGravity = parameters.mGravity;

    mRelaxationEpsilon = parameters.mRelaxationEpsilon;
    mSCorrDeltaQ = parameters.mSCorrDeltaQ;
    mSCorrK = parameters.mSCorrK;
    mSCorrN = parameters.mSCorrN;

    mCXsph = parameters.mCXsph;
    mEpsilonVorticity = parameters.mEpsilonVorticity;

    mTimeStep = parameters.mTimeStep;
    mSubsteps = parameters.mSubsteps;

    for (const IParticlesData* data : parameters.mParticlesData) {
        data->addParticlesData(this);
    }

    std::fill(mVelocities.begin(), mVelocities.end(), glm::vec3(0));
    std::fill(mPositionsStar.begin(), mPositionsStar.end(), glm::vec3(0));
    std::fill(mLambdas.begin(), mLambdas.end(), static_cast<float>(0));
    std::fill(mPressures.begin(), mPressures.end(), glm::vec3(0));
    mNeighbors = std::vector<std::vector<int>>(mNumParticles, std::vector<int>{});
    mCellSize = mKernelRadius;

    mGridX = static_cast<float>(mX / mCellSize) + 1;
    mGridY = static_cast<float>(mY / mCellSize) + 1;
    mGridZ = static_cast<float>(mZ / mCellSize) + 1;

    mNumGridCells = mGridX * mGridY * mGridZ;
    mUniformGrid = std::vector<std::vector<int>>(mNumGridCells, std::vector<int>{});

    //mMass = 3000.0f / mNumParticles;
    //mSCorrK = mMass * 1.0e-04f;
}

#define CHECK_NAN_VEC(V) \
    assert(!glm::isnan(V.x) || !glm::isnan(V.y) || !glm::isnan(V.z) && "the vector has nan comp"); \
    assert(!glm::isinf(V.x) || !glm::isinf(V.y) || !glm::isinf(V.z) && "the vector has inf comp");
    

#define CHECK_NAN_VAL(V) \
    assert(!glm::isnan(V) && "the value is nan"); \
    assert(!glm::isinf(V) && "the value is inf");


inline float Engine::s_coor(float rl) {
    float result = static_cast<float>(0);
    float W = mCubicKernel.W(mSCorrDeltaQ);
    if (W > 0.001f) { //glm::epsilon<float>()
        result = -mSCorrK * std::pow(mCubicKernel.W(rl) / W, mSCorrN);
    }
    CHECK_NAN_VAL(result);
    return result;
}

inline float Engine::resolve_collision(float value, float min, float max) {

    if (value <= min) {
        return mEpsilonCollision;
    }

    if (value > max) {
        return max - mEpsilonCollision;
    }

    return value;
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


void Engine::step() {

    //integration
    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] += mGravity * mMass * mTimeStep;
        CHECK_NAN_VEC(mVelocities[i]);
        mPositionsStar[i] = mPositions[i] + mVelocities[i] * mTimeStep; //prediction
        CHECK_NAN_VEC(mPositionsStar[i]);
    }

    findNeighborsUniformGrid();

    for (int s = 0; s < mSubsteps; s++) {

        //solve pressure
        for (int i = 0; i < mNumParticles; i++) {

            float densitiy = 0.0;
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                float len = glm::length(ij);
                densitiy += mMass * mCubicKernel.W(len);
                CHECK_NAN_VEC(ij);
            }
            densitiy += mMass * mCubicKernel.W(0.0f);

            //equation 1
            float constraint_i = (densitiy / mRestDensity) - static_cast<float>(1);
            float constraint_gradient_sum = static_cast<float>(0);
            glm::vec3 grad_current_p = glm::vec3(0.0);

            CHECK_NAN_VAL(constraint_i);
            
            //equation 8
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 temp = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                glm::vec3 neighbor_grad = -(mMass / mRestDensity) * mCubicKernel.WGrad(temp);
                CHECK_NAN_VEC(neighbor_grad);
                constraint_gradient_sum += glm::dot(neighbor_grad, neighbor_grad);
                grad_current_p -= neighbor_grad;
            }

            CHECK_NAN_VEC(grad_current_p);
            constraint_gradient_sum += glm::dot(grad_current_p, grad_current_p);

            mLambdas[i] = static_cast<float>(0);
            if (constraint_gradient_sum > 0.0f) {
                mLambdas[i] = -constraint_i / (constraint_gradient_sum + mRelaxationEpsilon);
                CHECK_NAN_VAL(mLambdas[i]);
            }

        }

        for (int i = 0; i < mNumParticles; i++) {

            //equation 13 (applying pressure force correction)
            //glm::vec3 pressure_force = glm::vec3(0.0);
            mPressures[i] = glm::vec3(0);
            for (int j = 0; j < mNeighbors[i].size(); j++) {
                glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
                CHECK_NAN_VEC(ij);
                mPressures[i] += (mLambdas[i] + mLambdas[j] + s_coor(glm::length(ij))) * mCubicKernel.WGrad(ij); //
                CHECK_NAN_VEC(mPressures[i]);
            }

            mPressures[i] *= (1.0f / (mRestDensity * mMass));
            
        }

        for (int i = 0; i < mNumParticles; i++) {
             mPositionsStar[i] += mPressures[i];
        }

        for (int i = 0; i < mNumParticles; i++) {
            glm::vec3 dp = glm::vec3(0.0);
            dp += mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(1, 0, 0), glm::vec3(0, 0, 0), mPositionsStar[i], mParticleRadius);
            dp += mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(-1, 0, 0), glm::vec3(mX, 0, 0), mPositionsStar[i], mParticleRadius);
            dp += mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 1, 0), glm::vec3(0, 0, 0), mPositionsStar[i], mParticleRadius);
            dp += mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, -1, 0), glm::vec3(0, mY, 0), mPositionsStar[i], mParticleRadius);
            dp += mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 0, 1), glm::vec3(0, 0, 0), mPositionsStar[i], mParticleRadius);
            dp += mBoundaryCollisionCoeff * solve_boundary_collision_constraint(glm::vec3(0, 0, -1), glm::vec3(0, 0, mZ), mPositionsStar[i], mParticleRadius);
            mPositionsStar[i] += dp;
            //mPositionsStar[i] = glm::clamp(mPositionsStar[i], mAABB.min + 2.0f * mParticleRadius, mAABB.max - 2.0f * mParticleRadius);
        }

    }

    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] = (mPositionsStar[i] - mPositions[i]) / mTimeStep;
        mPositions[i] = mPositionsStar[i];
    }

        /*
        //update prediction
        //mPositionsStar[i] += pressure_force;
        //CHECK_NAN_VEC(mPositionsStar[i]);

        //update velocity
        mPositionsStar[i] = glm::clamp(mPositionsStar[i], mAABB.min + mParticleRadius, mAABB.max - mParticleRadius);
        CHECK_NAN_VEC(mPositionsStar[i]);

        mVelocities[i] = (mPositionsStar[i] - mPositions[i]) / mTimeStep;
        
        CHECK_NAN_VEC(mVelocities[i]);

        mPositions[i] = mPositionsStar[i];
        //mPositions[i] = glm::clamp(mPositions[i], mAABB.min + mParticleRadius, mAABB.max - mParticleRadius);
        */
    
}

void Engine::resize(size_t newSize) {
    mPositionsStar.resize(newSize);
    mPositions.resize(newSize);
    mVelocities.resize(newSize);
    mLambdas.resize(newSize);
    mColors.resize(newSize);
    mPressures.resize(newSize);
}

inline int Engine::get_cell_id(glm::vec3 position) {
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

inline bool Engine::check_index(int i, int min, int max) {
    return (i >= min && i < max);
}

void Engine::findNeighborsUniformGrid() {

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

                for (int y = -1; y <= 1; y++) {
                    for (int x = -1; x <= 1; x++) {
                        for (int z = -1; z <= 1; z++) {
                            
                            if (
                                check_index(xx + x, 0, mGridX) == false ||
                                check_index(yy + y, 0, mGridY) == false ||
                                check_index(zz + z, 0, mGridZ) == false
                            )
                            {
                                continue;
                            }

                            int neighbor_cell_id =
                                (yy + y) * mGridX * mGridZ +
                                (xx + x) * mGridZ + 
                                (zz + z);

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
                                    if (glm::dot(tmp, tmp) <= mKernelRadius * mKernelRadius) {
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


void Engine::getParameters(EngineParameters& out) const {

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

}

const std::vector<glm::vec3>& Engine::getPositions() const {
    return mPositions;
}

const std::vector<glm::vec4>& Engine::getColors() const {
    return mColors;
}



}
