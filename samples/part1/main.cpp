#include <iostream>

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

    Levek::ArcballCamera camera;
    camera.setProjection(glm::perspective(glm::radians(45.0f), (resolutionX * 1.0f) / (1.0f * resolutionY), 0.01f, 1000.0f));
    camera.setRotation(5.34f);
    camera.setElevation(-0.59f);
    camera.setTarget({9.260883f, 0.000000f, 5.517349f});
    camera.setViewDistance(35.0f);

    Levek::SkyBoxPipelineState skybox;
    Levek::GroundPipelineState ground(modelLoader, 150.0f, false);

    Eloy::EngineParameters parameters;
    Eloy::AABBParticlesData aabb1({0.2, 0.2, 0.2}, {10, 10, 10}, {1, 0, 0, 1});
    parameters.mParticlesData.push_back(&aabb1);

    Eloy::Engine particleEngine(parameters);
    
    Eloy::ErrorCode err = Eloy::ELOY_ERROR_OK;
    Eloy::ParticlesPipelineSate particleRendering(engine, particleEngine.getPositions(), particleEngine.getColors(), err);

    while (!windowController->exit() && !inputController->isKeyPressed(Levek::LEVEK_KEY_Q)) {
        renderer->clear();
        camera.updateCameraOrientation(inputController, windowController);
        camera.updateCameraTargetWASD(inputController, windowController->getDeltaTime());
            
        skybox.draw(renderer, camera.getView(), camera.getProjection());
        ground.draw(renderer, camera.getViewProjection());
        
        particleEngine.step(0.01f);
        particleRendering.updatePositions(particleEngine.getPositions());
        particleRendering.updateColors(particleEngine.getColors());

        particleRendering.setUniforms(
            camera.getViewProjection(),
            camera.getProjection(),
            camera.getView(),
            camera.getViewInv(),
            glm::vec3(0, -1, 0),
            parameters.mParticuleRadius
        );
        particleRendering.draw(renderer);

        inputController->poll();
        windowController->swapBuffers();
    }

    delete engine;
    return 0;
}   