#pragma once

#include "Engine.hpp"

//#ifndef LEVEK_INCLUDE_IMGUI
//#define LEVEK_INCLUDE_IMGUI
//#endif
#include "LevekGL.hpp"
#include "imgui.h"

namespace Eloy {
class EngineImGui {
    Engine& engine;
public:
    EngineImGui(Engine& engine): engine(engine) {};
    bool imgui() {
        bool quit = false;
        ImGui::BeginTabBar("Engine");
        if (ImGui::BeginTabItem("Parameters")) {

            ImGui::Text("%d particles", engine.mNumParticles);
            ImGui::Text("%d cells", engine.mNumGridCells);
            ImGui::Text("particle radius %.3f", engine.mParticleRadius);
            ImGui::Text("particle diameter %.3f", engine.mParticleDiameter);

            ImGui::SliderFloat("boundary coeff", &engine.mBoundaryCollisionCoeff, 0.01f, 5.0f, "%.3f");

            ImGui::InputFloat("kernel radius", &engine.mKernelRadius, 0.01f, 5.0f, "%.3f");
            ImGui::SliderFloat("kernel factor", &engine.mkernelFactor, 0.0001f, 5.0f, "%.3f");
            engine.mCubicKernel.mFactor = engine.mkernelFactor;
            ImGui::SliderFloat("rest density", &engine.mRestDensity, 0.01f, 1500.0f, "%.3f");
            ImGui::SliderFloat("mass", &engine.mMass, 0.001f, 5.0f, "%.3f");
            ImGui::SliderFloat("gravity (-y)", &engine.mGravity.y, -50.01f, -0.1f, "%.3f");

            ImGui::SliderFloat("relaxation", &engine.mRelaxationEpsilon, 0.01f, 100.0f, "%.3f");

            ImGui::SliderInt("subsetps", &engine.mSubsteps, 1, 10, "%d");
            
            ImGui::SliderFloat("SCoorDeltaQ", &engine.mSCorrDeltaQ, 0.01f, 5.0f, "%.3f");
            ImGui::SliderFloat("SCoorK", &engine.mSCorrK, 0.00000001f, 0.1f, "%.3f");
            ImGui::SliderFloat("SCoorN", &engine.mSCorrN, 0.01f, 5.0f, "%.3f");

            ImGui::SliderFloat("Xsph", &engine.mCXsph, 0.01f, 5.0f, "%.3f");
            ImGui::SliderFloat("epsilon vorticity", &engine.mEpsilonVorticity, 0.01f, 5.0f, "%.3f");
            ImGui::SliderFloat("time step", &engine.mTimeStep, 0.001f, 0.08f, "%.3f");
            quit = ImGui::Button("reset");

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
        return quit;
    }
};
}