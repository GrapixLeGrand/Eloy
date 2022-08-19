#include "Eloy.hpp"

#include "glm/gtc/constants.hpp"

namespace Eloy {    

Engine::Engine(const EngineParameters& parameters) {

    mX = parameters.mX;
    mY = parameters.mY;
    mZ = parameters.mZ;

    mParticleRadius = parameters.mParticuleRadius;
    mParticleDiameter = parameters.mParticuleRadius * static_cast<float>(2);
    mKernelRadius = static_cast<float>(3.1) * mParticleRadius;
    mkernelFactor = parameters.mKernelFactor;

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
}

inline float Engine::s_coor(float rl) {
    return -mSCorrK * std::pow(mCubicKernel.W(rl) / mCubicKernel.W(mSCorrDeltaQ), mSCorrN);
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

void Engine::step(float dt) {

    dt = glm::clamp(dt, 0.001f, 0.01f);

    //integration
    for (int i = 0; i < mNumParticles; i++) {
        mVelocities[i] += mGravity * mMass * dt;
        mPositionsStar[i] = mPositions[i] + mVelocities[i] * dt; //prediction
    }

    //find_mNeighbors_uniform_grid(simulation);

    //solve pressure
    for (int i = 0; i < mNumParticles; i++) {

        float densitiy = 0.0;
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            float len = glm::length(ij);
            densitiy += mMass * mCubicKernel.W(len);
        }
        densitiy += mMass * mCubicKernel.W(0.0f);

        //equation 1
        float constraint_i = (densitiy / mRestDensity) - static_cast<float>(1);
        float constraint_gradient_sum = static_cast<float>(0);
        glm::vec3 grad_current_p = glm::vec3(0.0);

        //equation 8
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            glm::vec3 temp = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            glm::vec3 neighbor_grad = -(mMass / mRestDensity) * mCubicKernel.WGrad(temp);
            constraint_gradient_sum += glm::dot(neighbor_grad, neighbor_grad);
            grad_current_p -= neighbor_grad;
        }

        constraint_gradient_sum += glm::dot(grad_current_p, grad_current_p);

        mLambdas[i] = static_cast<float>(0);
        if (constraint_gradient_sum > 0.0f) {
            mLambdas[i] = -constraint_i / (constraint_gradient_sum + mRelaxationEpsilon);
        }

    }

    for (int i = 0; i < mNumParticles; i++) {

        //equation 13 (applying pressure force correction)
        glm::vec3 pressure_force = glm::vec3(0.0);
        for (int j = 0; j < mNeighbors[i].size(); j++) {
            glm::vec3 ij = mPositionsStar[i] - mPositionsStar[mNeighbors[i][j]];
            pressure_force += (mLambdas[i] + mLambdas[j] + s_coor(glm::length(ij))) * mCubicKernel.WGrad(ij);
        }

        pressure_force /= mRestDensity;

        //update prediction
        mPositionsStar[i] += pressure_force;

        //update velocity
        mPositionsStar[i] = glm::clamp(mPositionsStar[i], glm::vec3(mParticleRadius), glm::vec3(mX - mParticleRadius, mY - mParticleRadius, mZ - mParticleRadius));

        mVelocities[i] = (mPositionsStar[i] - mPositions[i]) / mTimeStep;
        mPositions[i] = mPositionsStar[i];

    }
}


inline int Engine::get_cell_id(glm::vec3 position) {
    //position = glm::clamp(position, glm::vec3(simulation->mCellSize * 0.5f), glm::vec3(simulation->domainX - simulation->mCellSize * 0.5f, simulation->domainY - simulation->mCellSize * 0.5f, simulation->domainZ - simulation->mCellSize * 0.5f));
    position /= mCellSize;
    int cell_id =
            ((int) position.y) * mGridX * mGridZ +
            ((int) position.x) * mGridZ +
            ((int) position.z); 
    //cell_id = glm::clamp(cell_id, 0, simulation->mNumGridCells - 1);
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
        glm::vec3& position = mPositionsStar[i];
        mUniformGrid[cell_id].push_back(i);
    }

    for (int i = 0; i < mNumParticles; i++) {
        int cell_id = get_cell_id(mPositionsStar[i]);
        glm::vec3& position = mPositionsStar[i];
        mUniformGrid[cell_id].push_back(i);
    }

    for (int yy = 0; yy < mGridY; yy++) {
        for (int xx = 0; xx < mGridX; xx++) {
            for (int zz = 0; zz < mGridZ; zz++) {

                int cell_id = 
                    yy * mGridX * mGridZ +
                    xx * mGridZ + 
                    zz;

                std::vector<int>& current_cell_indices = mUniformGrid[cell_id];

                if (current_cell_indices.empty() == true) {
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

                            std::vector<int>& neighbor_cell_indices = mUniformGrid[neighbor_cell_id];

                            if (neighbor_cell_indices.empty() == true) {
                                continue;
                            }

                            for (int i = 0; i < current_cell_indices.size(); i++) {
                                const int current_index = current_cell_indices[i];
                                /*if (current_index >= ptr_solid_start) {
                                    continue;
                                }*/
                                const glm::vec3& self = mPositionsStar[current_index];
                                for (int j = 0; j < neighbor_cell_indices.size(); j++) {
                                    const int neighbor_index = neighbor_cell_indices[j];
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

}
