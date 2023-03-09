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
    camera.setRotation(4.14f);
    camera.setElevation(-0.18f);
    camera.setTarget({9.260883f, 0.000000f, 5.517349f});
    camera.setViewDistance(15.0f);

    Levek::SkyBoxPipelineState skybox;
    Levek::GroundPipelineState ground(modelLoader, 150.0f, false);

    Eloy::PBDSolverParameters parameters;
    //parameters.mParticuleRadius = 0.25f;
    //glm::vec3 min_pos = {0.5, 0.5, 0.5};
    //glm::vec3 max_pos = {5, 10, 5};
    //glm::vec3 offset = {3, 0, 3};
    glm::vec3 min_pos = {0.5, 0.5, 0.5};
    glm::vec3 max_pos = {2, 2.5, 2};
    glm::vec3 offset = {0.01, 0.01, 0.1};
    Eloy::AABBParticlesData aabb1(min_pos + offset, max_pos + offset, {1, 0, 0, 1});

    parameters.mParticlesData.push_back(&aabb1);
    
    Eloy::FluidBehavior moveFluid;
    //parameters.mPreStepCallBack = moveFluid.getFun();
    parameters.behaviors.push_back(&moveFluid);

    Eloy::PBDVerletSolver verletSolver(parameters);
    Eloy::PBDPackedSolver packedSolver(parameters);
    Eloy::PBDSolver* solver = &verletSolver;
    
    Eloy::ErrorCode err = Eloy::ELOY_ERROR_OK;
    Eloy::ParticlesPipelineSate particleRendering(engine, solver->getPositions(), solver->getColors(), err);
    //Eloy::PBDSolverImGui engineImGui;

    Levek::FrameBuffer mainFb(resolutionX, resolutionY);
    Levek::Texture sceneResult(resolutionX, resolutionY, Levek::TextureParameters::TextureType::RGB);
    Levek::Texture sceneDepthStencil(resolutionX, resolutionY, Levek::TextureParameters::TextureType::DEPTH);
    

    mainFb.addAttachment(&sceneResult, Levek::FrameBufferProperties::AttachementType::COLOR);
    //mainFb.addColorAttachment(&sceneResult, 0);
    mainFb.addAttachment(&sceneDepthStencil, Levek::FrameBufferProperties::AttachementType::DEPTH);

    int selectedSolver = 0;
    Eloy::PBDSolver* solvers[2] = { &verletSolver, &packedSolver };

    while (!windowController->exit() && !inputController->isKeyPressed(Levek::LEVEK_KEY_Q)) {
        renderer->clear();
        //mainFb.clear();
        //sceneResult.clear({0, 0, 0, 0});
        //sceneDepthStencil.clear(1.0f);

        camera.updateCameraOrientation(inputController, windowController);
        camera.updateCameraTargetWASD(inputController, windowController->getDeltaTime());
        sceneResult.clear(glm::uvec4{0, 0, 0, 0});
        sceneDepthStencil.clear(1.0f);

        skybox.draw(&mainFb, renderer, camera.getView(), camera.getProjection());
        ground.draw(&mainFb, renderer, camera.getViewProjection());
        
        solver->step();
        particleRendering.updatePositions(solver->getPositions());
        particleRendering.updateColors(solver->getColors());

        particleRendering.setUniforms(
            camera.getViewProjection(),
            camera.getProjection(),
            camera.getView(),
            camera.getViewInv(),
            glm::vec3(0, -1, 0),
            parameters.getDiameter()
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

        static const char* solverTypes[]{"verlet", "packed"};
        int saveSelectedSolver = selectedSolver;
        ImGui::Combo("solver", &selectedSolver, solverTypes, IM_ARRAYSIZE(solverTypes));

        if (solver->imgui()) {
            solver->reset();
        }

        if (saveSelectedSolver != selectedSolver) {
            solver = solvers[selectedSolver];
            solver->reset();
        }

        ImGui::End();

        moveFluid.imGui();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        

        /*std::cout << "left nobe (" << 
            inputController->gamePadAxis(Levek::Joystick::LEVEK_JOYSTICK_1, 0) << ", " <<  
            inputController->gamePadAxis(Levek::Joystick::LEVEK_JOYSTICK_1, 1) << std::endl; */
        /*std::cout << "left nobe (" << 
            inputController->gamePadAxis(Levek::Joystick::LEVEK_JOYSTICK_1, 0) << ", " <<  
            inputController->gamePadAxis(Levek::Joystick::LEVEK_JOYSTICK_1, 1) << std::endl; 
        */
        
        moveFluid.setAnalog(0, {inputController->gamePadAxis(Levek::Joystick::LEVEK_JOYSTICK_1, 0), inputController->gamePadAxis(Levek::Joystick::LEVEK_JOYSTICK_1, 1)});
        moveFluid.setAnalog(1, {inputController->gamePadAxis(Levek::Joystick::LEVEK_JOYSTICK_1, 2), inputController->gamePadAxis(Levek::Joystick::LEVEK_JOYSTICK_1, 3)});
        moveFluid.setGravity(!inputController->gamePadButton(Levek::Joystick::LEVEK_JOYSTICK_1, Levek::GamePadButton::LEVEK_BUTTON_A));


        inputController->poll();
        windowController->swapBuffers();
    }

    delete engine;
    return 0;
}   