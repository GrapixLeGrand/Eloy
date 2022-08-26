#pragma once

#include "Engine.hpp"

//#ifndef LEVEK_INCLUDE_IMGUI
//#define LEVEK_INCLUDE_IMGUI
//#endif
#include "LevekGL.hpp"
#include "imgui.h"
#include "imGuIZMO.quat/imGuIZMOquat.h"

namespace Eloy {
class EngineImGui {
    Engine& engine;
    int selectedSolver = 1;
    int selectedNeighbor = 1;
public:
    EngineImGui(Engine& engine): engine(engine) {};
    bool imgui() {
        bool quit = false;
        ImGui::BeginTabBar("Engine");
        if (ImGui::BeginTabItem("Parameters")) {
            

            ImGui::Text("%d particles %lf ms", engine.mNumParticles, engine.mSolverMs);
            ImGui::Text("%d cells %lf ms", engine.mNumGridCells, engine.mNeighborMs);

            ImGui::Text("simulation (everything) %lf ms (%lf fps)", engine.mSolverFullMs, (1.0 / engine.mSolverFullMs) * 1000.0);
            
            static const char* solverTypes[]{"single-thread", "multi-threaded"}; 
            ImGui::Combo("solver", &selectedSolver, solverTypes, IM_ARRAYSIZE(solverTypes));
            engine.mSolverMode = (Engine::SolverMode) selectedSolver;

            static const char* neighborTypes[]{"basic-verlet", "minimalist"}; 
            ImGui::Combo("neighbors", &selectedNeighbor, neighborTypes, IM_ARRAYSIZE(neighborTypes));
            engine.mNeighborMode = (Engine::NeighborMode) selectedNeighbor;


            ImGui::Text("particle radius %.3f", engine.mParticleRadius);
            ImGui::Text("particle diameter %.3f", engine.mParticleDiameter);

            ImGui::SliderFloat("boundary coeff", &engine.mBoundaryCollisionCoeff, 0.01f, 5.0f, "%.3f");

            ImGui::InputFloat("kernel radius", &engine.mKernelRadius, 0.01f, 5.0f, "%.3f");
            ImGui::SliderFloat("kernel factor", &engine.mkernelFactor, 0.0001f, 5.0f, "%.3f");
            engine.mCubicKernel.mFactor = engine.mkernelFactor;
            ImGui::SliderFloat("rest density", &engine.mRestDensity, 0.01f, 1500.0f, "%.3f");
            ImGui::SliderFloat("mass", &engine.mMass, 0.001f, 5.0f, "%.3f");
            ImGui::SliderFloat("gravity (-y)", &engine.mGravity.y, -50.01f, -0.1f, "%.3f");

            ImGui::SliderFloat("relaxation", &engine.mRelaxationEpsilon, 0.01f, 1e6f, "%.3f");

            ImGui::SliderInt("subsetps", &engine.mSubsteps, 1, 10, "%d");
            
            ImGui::SliderFloat("SCoorDeltaQ", &engine.mSCorrDeltaQ, 0.001f, 0.5f, "%.3f");
            ImGui::SliderFloat("SCoorK", &engine.mSCorrK, 0.00000001f, 0.1f, "%.3f");
            ImGui::SliderFloat("SCoorN", &engine.mSCorrN, 0.01f, 5.0f, "%.3f");

            ImGui::SliderFloat("Xsph", &engine.mCXsph, 0.0001f, 1.0f, "%.3f");
            ImGui::SliderFloat("epsilon vorticity", &engine.mEpsilonVorticity, 0.01f, 1.0f, "%.3f");
            ImGui::SliderFloat("time step", &engine.mTimeStep, 0.001f, 0.08f, "%.3f");
            quit = ImGui::Button("reset");

            vgm::Vec3 v;
            v.x = engine.mGravity.x;
            v.y = engine.mGravity.y;
            v.z = engine.mGravity.z;
            ImGui::gizmo3D("##Dir1", v, 100.0f);
            engine.mGravity = { v.x, v.y, v.z };

            if (ImGui::Button("reset gravity")) {
                engine.mGravity = { 0, -10, 0 };
            }

            if (ImGui::Button("no gravity")) {
                engine.mGravity = { 0, 0, 0 };
            }

            if (ImGui::Button("save state")) {
                engine.writeParticlesToJson(ELOY_BUILD_DIRECTORY"/save.json");
            }

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
        return quit;
    }
};
}