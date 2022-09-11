#include <iostream>

#define LEVEK_INCLUDE_IMGUI
#include "LevekGL.hpp"
#include "Eloy.hpp"

int resolutionX = 1920;
int resolutionY = 1080;

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

    Eloy::EngineParameters parameters;
    //parameters.mParticuleRadius = 0.25f;
    glm::vec3 min_pos = {0.5, 0.5, 0.5};
    glm::vec3 max_pos = {5, 10, 5};
    glm::vec3 offset = {3, 3, 3};
    Eloy::AABBParticlesData aabb1(min_pos + offset, max_pos + offset, {1, 0, 0, 1});

    parameters.mParticlesData.push_back(&aabb1);

    Eloy::Engine particleEngine(parameters);
    Eloy::EngineImGui engineImGui(particleEngine);
    
    Eloy::ErrorCode err = Eloy::ErrorCode::ELOY_ERROR_OK;
    float imguiScaleFactor = 0.6f;
    Eloy::ParticlesRenderingSpf spfParticleRendering(engine, particleEngine.getPositions(), particleEngine.getColors(), err);


    float blurFilterRadius = 8.0f;
    float blurScale = 0.001f;
    float blurDepthFallOff = 0.001f;

    while (!windowController->exit() && !inputController->isKeyPressed(Levek::LEVEK_KEY_Q)) {
        renderer->clear();

        camera.updateCameraOrientation(inputController, windowController);
        camera.updateCameraTargetWASD(inputController, windowController->getDeltaTime());

        particleEngine.step();
        spfParticleRendering.updatePositions(particleEngine.getPositions());
        spfParticleRendering.updateColors(particleEngine.getColors());

        sceneResult.clear(glm::uvec4(0, 0, 0, 0));
        sceneDepthStencil.clear(1.0f);

        skybox.draw(&mainFb, renderer, camera.getView(), camera.getProjection());
        ground.draw(&mainFb, renderer, camera.getViewProjection());
        
        spfParticleRendering.draw(
            sceneResult,
            camera,
            glm::vec3{0, -1, 0},
            0.2f
        );

        renderer->draw(spfParticleRendering.getScene(), {0.0, 0.0}, {1, 1});

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::Begin("Main");

        std::string fps = std::to_string((int)(1.0f / windowController->getDeltaTime()));
        fps += " fps";
        ImGui::Text(fps.c_str());

        //ImGui::Image((void*)(intptr_t)sceneResult.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));
        spfParticleRendering.imGui(imguiScaleFactor);

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