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

            ImGui::InputFloat("kernel radius", &engine.mKernelRadius, 0.01f, 5.0f, "%.3f");
            ImGui::SliderFloat("kernel factor", &engine.mkernelFactor, 0.0001f, 5.0f, "%.3f");
            engine.mCubicKernel.mFactor = engine.mkernelFactor;
            ImGui::InputFloat("rest density", &engine.mRestDensity, 0.01f, 200.0f, "%.3f");
            ImGui::InputFloat("mass", &engine.mMass, 0.01f, 100.0f, "%.3f");
            ImGui::InputFloat("gravity (-y)", &engine.mGravity.y, -50.01f, -0.1f, "%.3f");

            ImGui::InputFloat("relaxation", &engine.mRelaxationEpsilon, 0.01f, 1000.0f, "%.3f");
            
            ImGui::InputFloat("SCoorDeltaQ", &engine.mSCorrDeltaQ, 0.01f, 5.0f, "%.3f");
            ImGui::InputFloat("SCoorK", &engine.mSCorrK, 0.01f, 5.0f, "%.3f");
            ImGui::InputFloat("SCoorN", &engine.mSCorrN, 0.01f, 5.0f, "%.3f");

            ImGui::InputFloat("Xsph", &engine.mCXsph, 0.01f, 5.0f, "%.3f");
            ImGui::InputFloat("epsilon vorticity", &engine.mEpsilonVorticity, 0.01f, 5.0f, "%.3f");
            quit = ImGui::Button("reset");

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
        return quit;
    }
};
}