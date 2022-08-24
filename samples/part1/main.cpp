#include <iostream>

#define LEVEK_INCLUDE_IMGUI
#include "LevekGL.hpp"
#include "Eloy.hpp"

int resolutionX = 1920;
int resolutionY = 1080;

int main(int argc, char** argv) {
    
    Levek::RenderingEngine* engine = new Levek::RenderingEngine(resolutionX, resolutionY, true);
    Levek::WindowController* windowController = engine->getWindowController();
    Levek::InputController* inputController = engine->getInputController();
    Levek::Renderer* renderer = engine->getRenderer();
    Levek::ModelLoader* modelLoader = engine->getModelLoader();
    windowController->initImGui();

    Levek::ArcballCamera camera;
    camera.setProjection(glm::perspective(glm::radians(45.0f), (resolutionX * 1.0f) / (1.0f * resolutionY), 0.01f, 1000.0f));
    camera.setRotation(5.34f);
    camera.setElevation(-0.59f);
    camera.setTarget({9.260883f, 0.000000f, 5.517349f});
    camera.setViewDistance(35.0f);

    Levek::SkyBoxPipelineState skybox;
    Levek::GroundPipelineState ground(modelLoader, 150.0f, false);

    Eloy::EngineParameters parameters;
    //parameters.mParticuleRadius = 0.25f;
    glm::vec3 min_pos = {0.5, 0.5, 0.5};
    glm::vec3 max_pos = {5, 10, 5};
    glm::vec3 offset = {3, 3, 3};
    Eloy::AABBParticlesData aabb1(min_pos + offset, max_pos + offset, {1, 0, 0, 1});

    parameters.mParticlesData.push_back(&aabb1);

    Eloy::Engine particleEngine(parameters);
    
    Eloy::ErrorCode err = Eloy::ELOY_ERROR_OK;
    Eloy::ParticlesPipelineSate particleRendering(engine, particleEngine.getPositions(), particleEngine.getColors(), err);
    Eloy::EngineImGui engineImGui(particleEngine);

    while (!windowController->exit() && !inputController->isKeyPressed(Levek::LEVEK_KEY_Q)) {
        renderer->clear();
        camera.updateCameraOrientation(inputController, windowController);
        camera.updateCameraTargetWASD(inputController, windowController->getDeltaTime());
            
        skybox.draw(renderer, camera.getView(), camera.getProjection());
        ground.draw(renderer, camera.getViewProjection());
        
        particleEngine.step();
        particleRendering.updatePositions(particleEngine.getPositions());
        particleRendering.updateColors(particleEngine.getColors());

        particleRendering.setUniforms(
            camera.getViewProjection(),
            camera.getProjection(),
            camera.getView(),
            camera.getViewInv(),
            glm::vec3(0, -1, 0),
            particleEngine.getDiameter()
        );
        particleRendering.draw(renderer);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::Begin("Main");

        std::string fps = std::to_string((int)(1.0f / windowController->getDeltaTime()));
        fps += " fps";
        ImGui::Text(fps.c_str());

        if (engineImGui.imgui()) {
            particleEngine.getParameters(parameters);
            particleEngine = Eloy::Engine(parameters);
            //engineImGui = Eloy::EngineImGui(particleEngine);
        }

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        inputController->poll();
        windowController->swapBuffers();
    }

    delete engine;
    return 0;
}   