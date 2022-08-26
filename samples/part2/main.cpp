#include <iostream>

#define LEVEK_INCLUDE_IMGUI
#include "LevekGL.hpp"
#include "Eloy.hpp"

int resolutionX = 1280;
int resolutionY = 720;

int main(int argc, char** argv) {
    
    Levek::RenderingEngine* engine = new Levek::RenderingEngine(resolutionX, resolutionY);
    Levek::WindowController* windowController = engine->getWindowController();
    Levek::InputController* inputController = engine->getInputController();
    Levek::Renderer* renderer = engine->getRenderer();
    Levek::ModelLoader* modelLoader = engine->getModelLoader();
    windowController->initImGui();

    Levek::ArcballCamera camera;
    camera.setProjection(glm::perspective(glm::radians(45.0f), (resolutionX * 1.0f) / (1.0f * resolutionY), 0.01f, 1000.0f));
    camera.setRotation(4.04f);
    camera.setElevation(-0.57f);
    camera.setTarget({16.360883f, 0.000000f, 12.17349f});
    camera.setViewDistance(35.0f);

    Levek::SkyBoxPipelineState skybox;
    Levek::GroundPipelineState ground(modelLoader, 150.0f, false);

    Levek::FrameBuffer mainFb(resolutionX, resolutionY);
    Levek::Texture sceneResult(resolutionX, resolutionY, Levek::TextureParameters::TextureType::RGB);
    Levek::Texture sceneDepthStencil(resolutionX, resolutionY, Levek::TextureParameters::TextureType::DEPTH);
    

    mainFb.addAttachment(&sceneResult, Levek::FrameBufferProperties::AttachementType::COLOR);
    //mainFb.addColorAttachment(&sceneResult, 0);
    mainFb.addAttachment(&sceneDepthStencil, Levek::FrameBufferProperties::AttachementType::DEPTH);

    auto particleSavedState = Eloy::getParticlesStateFromSaveJson();
    Eloy::ErrorCode err = Eloy::ErrorCode::ELOY_ERROR_OK;
    Eloy::ParticlesPipelineSate particleRendering(engine, particleSavedState.first, particleSavedState.second, err);

    float imguiScaleFactor = 0.25f;

    while (!windowController->exit() && !inputController->isKeyPressed(Levek::LEVEK_KEY_Q)) {
        renderer->clear();
        //mainFb.clear();
        //sceneResult.clear({0, 0, 0, 0});
        //sceneDepthStencil.clear(1.0f);

        camera.updateCameraOrientation(inputController, windowController);
        camera.updateCameraTargetWASD(inputController, windowController->getDeltaTime());
        sceneResult.clear({0, 0, 0, 0});
        sceneDepthStencil.clear(1.0f);

        skybox.draw(&mainFb, renderer, camera.getView(), camera.getProjection());
        ground.draw(&mainFb, renderer, camera.getViewProjection());
        
        
        particleRendering.setUniforms(
            camera.getViewProjection(),
            camera.getProjection(),
            camera.getView(),
            camera.getViewInv(),
            glm::vec3(0, -1, 0),
            0.2f
        );
        particleRendering.draw(&mainFb, renderer);

        renderer->draw(&sceneResult, {0.0, 0.0}, {1, 1});

        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::Begin("Main");

        std::string fps = std::to_string((int)(1.0f / windowController->getDeltaTime()));
        fps += " fps";
        ImGui::Text(fps.c_str());

        ImGui::Image((void*)(intptr_t)sceneResult.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));
        ImGui::Image((void*)(intptr_t)sceneDepthStencil.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        inputController->poll();
        windowController->swapBuffers();
    }

    delete engine;
    return 0;
}   