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

    auto particleSavedState = Eloy::getParticlesStateFromSaveJson();
    Eloy::ErrorCode err = Eloy::ErrorCode::ELOY_ERROR_OK;
    Eloy::ParticlesPipelineSate particleRendering(engine, particleSavedState.first, particleSavedState.second, err);

    float imguiScaleFactor = 0.6f;

    //definition of buffers for spf rendering
    Levek::FrameBuffer screenSpaceFbPass1(resolutionX, resolutionY);
    Levek::Texture spfColor = Levek::Texture(resolutionX, resolutionY, Levek::TextureParameters::TextureType::RGBA_FLOAT);
    Levek::Texture spfDepthR(resolutionX, resolutionY, Levek::TextureParameters::TextureType::R_FLOAT);
    Levek::Texture spfDepth(resolutionX, resolutionY, Levek::TextureParameters::TextureType::DEPTH);

    screenSpaceFbPass1.addColorAttachment(&spfColor, 0);
    screenSpaceFbPass1.addColorAttachment(&spfDepthR, 1);
    screenSpaceFbPass1.addAttachment(&spfDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);
    

    // blur

    Levek::Texture spfDummyDepth(resolutionX, resolutionY, Levek::TextureParameters::TextureType::DEPTH);

    Levek::FrameBuffer screenSpaceFbBlurPass1(resolutionX, resolutionY);
    Levek::Texture spfBlurVertical(resolutionX, resolutionY, Levek::TextureParameters::TextureType::R_FLOAT);
    screenSpaceFbBlurPass1.addColorAttachment(&spfBlurVertical, 0);
    screenSpaceFbBlurPass1.addAttachment(&spfDummyDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);

    Levek::FrameBuffer screenSpaceFbBlurPass2(resolutionX, resolutionY);
    Levek::Texture spfBlurHorizontal(resolutionX, resolutionY, Levek::TextureParameters::TextureType::R_FLOAT);
    screenSpaceFbBlurPass2.addColorAttachment(&spfBlurHorizontal, 0);
    screenSpaceFbBlurPass2.addAttachment(&spfDummyDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);

    //end blur

    Levek::FrameBuffer screenSpaceFbPass2(resolutionX, resolutionY);
    Levek::Texture spfOutScene(resolutionX, resolutionY, Levek::TextureParameters::TextureType::RGBA_FLOAT);
    Levek::Texture spfNormal(resolutionX, resolutionY, Levek::TextureParameters::TextureType::RGBA_FLOAT);
    Levek::Texture spfDepth2(resolutionX, resolutionY, Levek::TextureParameters::TextureType::DEPTH);
    screenSpaceFbPass2.addColorAttachment(&spfOutScene, 0);
    screenSpaceFbPass2.addColorAttachment(&spfNormal, 1);
    screenSpaceFbPass2.addAttachment(&spfDepth2, Levek::FrameBufferProperties::AttachementType::DEPTH);


    Eloy::ParticlesRenderingSpf spfParticleRendering(engine, particleSavedState.first, particleSavedState.second, err);


    float blurFilterRadius = 8.0f;
    float blurScale = 0.001f;
    float blurDepthFallOff = 0.001f;

    while (!windowController->exit() && !inputController->isKeyPressed(Levek::LEVEK_KEY_Q)) {
        renderer->clear();

        camera.updateCameraOrientation(inputController, windowController);
        camera.updateCameraTargetWASD(inputController, windowController->getDeltaTime());
        sceneResult.clear(glm::uvec4(0, 0, 0, 0));
        sceneDepthStencil.clear(1.0f);

        spfColor.clear(glm::vec4(0, 0, 0, 0));
        spfDepthR.clear(0.0f);
        spfNormal.clear(glm::vec4(0, 0, 0, 0));
        spfDepth.clear(1.0f);
        spfDepth2.clear(1.0f);
        spfOutScene.clear(glm::vec4(0, 0, 0, 0));
        
        spfDummyDepth.clear(1.0f);
        

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
        

        //renderer->draw(&sceneResult, {0.0, 0.0}, {1, 1});

        spfParticleRendering.setUniformsPass1(
            camera.getViewProjection(),
            camera.getProjection(),
            camera.getView(),
            camera.getViewInv(),
            glm::vec3(0, -1, 0),
            0.2f
        );
        spfParticleRendering.drawPass1(&screenSpaceFbPass1, renderer);

        spfBlurVertical.clear(0.0f);
        spfDummyDepth.clear(1.0f);

        spfParticleRendering.setUniformsBlur(
            spfDepthR,
            {0, 1.0f / ((float)resolutionY) },
            blurFilterRadius,
            blurScale,
            blurDepthFallOff
        );

        spfParticleRendering.blurDepth(
            &screenSpaceFbBlurPass1,
            renderer
        );

        spfBlurHorizontal.clear(0.0f);
        spfDummyDepth.clear(1.0f);

        spfParticleRendering.setUniformsBlur(
            spfBlurVertical,
            {1.0f / resolutionX, 0 },
            blurFilterRadius,
            blurScale,
            blurDepthFallOff
        );

        spfParticleRendering.blurDepth(
            &screenSpaceFbBlurPass2,
            renderer
        );

        spfParticleRendering.setUniformsPass2(
            camera,
            glm::vec3(0, -1, 0),
            spfBlurHorizontal,
            sceneResult
        );

        spfParticleRendering.drawPass2(&screenSpaceFbPass2, renderer);

        renderer->draw(&sceneResult, {0.0, 0.0}, {1, 1});

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::Begin("Main");

        std::string fps = std::to_string((int)(1.0f / windowController->getDeltaTime()));
        fps += " fps";
        ImGui::Text(fps.c_str());

        ImGui::Text("Color");
        ImGui::Image((void*)(intptr_t)spfColor.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));
        

        ImGui::Text("depth view");
        ImGui::Image((void*)(intptr_t)spfDepthR.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));

        ImGui::Text("depth view V");
        ImGui::Image((void*)(intptr_t)spfBlurVertical.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));

        ImGui::SliderFloat("blurRadius", &blurFilterRadius, 1, 50);
        ImGui::SliderFloat("blurScale", &blurScale, 0.00001f, 100.0f);
        ImGui::SliderFloat("blurDepthFallOff", &blurDepthFallOff, 0.001f, 1000.0f);

        ImGui::Text("depth view H");
        ImGui::Image((void*)(intptr_t)spfBlurHorizontal.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));
        //ImGui::Text("blurred depth view");

        ImGui::Text("Normal (from depth)");
        ImGui::Image((void*)(intptr_t)spfNormal.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));
        
        ImGui::Image((void*)(intptr_t)spfOutScene.getId(), ImVec2(static_cast<float>(resolutionX) * imguiScaleFactor, static_cast<float>(resolutionY) * imguiScaleFactor), ImVec2(0, 1), ImVec2(1, 0));


        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        inputController->poll();
        windowController->swapBuffers();
    }

    delete engine;
    return 0;
}   