#include "PBDSolver.hpp"

#include "ParticlesData.hpp"
#include "nlohmann/json.hpp"

#include <fstream>

namespace Eloy {

PBDSolver::PBDSolver(const PBDSolverParameters& params) : mParameters(params), mParametersSave(params), mCubicKernel(CubicKernel(params.mKernelRadius, params.mKernelFactor)) {
    mAABB = AABB({0, 0, 0} , {mParameters.mX, mParameters.mY, mParameters.mZ});
    for (const IParticlesData* data : mParameters.mParticlesData) {
        data->addParticlesData(this);
    }
}


const std::vector<glm::vec3>& PBDSolver::getPositions() const {
    return mPositions;
}

const std::vector<glm::vec4>& PBDSolver::getColors() const {
    return mColors;
}

void PBDSolver::reset() {

    printf("reset of parent solver\n");

    mParameters = mParametersSave;
    mCubicKernel = CubicKernel(mParameters.mKernelRadius, mParameters.mKernelFactor);
    mAABB = AABB({0, 0, 0} , {mParameters.mX, mParameters.mY, mParameters.mZ});

    mNumParticles = 0;
    mVelocities.clear();
    mPositions.clear();
    mColors.clear();

    for (const IParticlesData* data : mParameters.mParticlesData) {
        data->addParticlesData(this);
    }
}

bool PBDSolver::imgui() {
    bool quit = false;
    ImGui::BeginTabBar("PBD Solver");
    if (ImGui::BeginTabItem("PBD solver parameters")) {

        ImGui::Text("particle radius %.3f", mParameters.mParticleRadius);
        ImGui::Text("particle diameter %.3f", 2.0f*mParameters.mParticleRadius);

        ImGui::SliderFloat("boundary coeff", &mParameters.mBoundaryCollisionCoeff, 0.01f, 5.0f, "%.3f");

        ImGui::InputFloat("kernel radius", &mParameters.mKernelRadius, 0.01f, 5.0f, "%.3f");
        ImGui::SliderFloat("kernel factor", &mParameters.mKernelFactor, 0.0001f, 5.0f, "%.3f");
        mCubicKernel.mFactor = mParameters.mKernelFactor;
        ImGui::SliderFloat("rest density", &mParameters.mRestDensity, 0.01f, 1500.0f, "%.3f");
        ImGui::SliderFloat("mass", &mParameters.mMass, 0.001f, 5.0f, "%.3f");
        ImGui::SliderFloat("gravity (-y)", &mParameters.mGravity.y, -50.01f, -0.1f, "%.3f");

        ImGui::SliderFloat("relaxation", &mParameters.mRelaxationEpsilon, 0.01f, 1e6f, "%.3f");

        ImGui::SliderInt("subsetps", &mParameters.mSubsteps, 1, 10, "%d");
        
        ImGui::SliderFloat("SCoorDeltaQ", &mParameters.mSCorrDeltaQ, 0.001f, 0.5f, "%.3f");
        ImGui::SliderFloat("SCoorK", &mParameters.mSCorrK, 0.00000001f, 0.1f, "%.3f");
        ImGui::SliderFloat("SCoorN", &mParameters.mSCorrN, 0.01f, 5.0f, "%.3f");

        ImGui::SliderFloat("Xsph", &mParameters.mCXsph, 0.0001f, 1.0f, "%.3f");
        ImGui::SliderFloat("epsilon vorticity", &mParameters.mEpsilonVorticity, 0.01f, 1.0f, "%.3f");
        ImGui::SliderFloat("time step", &mParameters.mTimeStep, 0.001f, 0.08f, "%.3f");
        quit = ImGui::Button("reset");

        vgm::Vec3 v;
        v.x = mParameters.mGravity.x;
        v.y = mParameters.mGravity.y;
        v.z = mParameters.mGravity.z;
        ImGui::gizmo3D("##Dir1", v, 100.0f);
        mParameters.mGravity = { v.x, v.y, v.z };

        if (ImGui::Button("reset gravity")) {
            mParameters.mGravity = { 0, -10, 0 };
        }

        if (ImGui::Button("no gravity")) {
            mParameters.mGravity = { 0, 0, 0 };
        }
        ImGui::EndTabItem();
        ImGui::EndTabBar();
    }
    return quit;
}


void PBDSolver::writeParticlesToJson(const std::string& filepath) {
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

}