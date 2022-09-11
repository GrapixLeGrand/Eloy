#pragma once

#include "ParticlesRendering.hpp"
#include "imgui.h"

namespace Eloy {

class ParticlesRenderingSpf : public ParticlesPipelineSate {

    int resolutionX = 0;
    int resolutionY = 0;

    Levek::Shader spfShaderPass1 = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass1.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass1.frag"
    );
    

    Levek::Shader spfShaderBlur = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidBilateralFilter.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidBilateralFilter.frag"
    );

    Levek::Shader spfShaderPass2 = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass2.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass2.frag"
    );

    Levek::Shader spfShaderThickness = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidThickness.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidThickness.frag"
    );

    Levek::Shader spfShaderThicknessBlur = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidGaussianBlur.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidGaussianBlur.frag"
    );

    //pass 1
    Levek::FrameBuffer screenSpaceFbPass1;
    Levek::Texture spfColor;
    Levek::Texture spfDepthPass1;
    Levek::Texture spfDepth;

    //blur passes
    Levek::Texture spfDummyDepth;
    Levek::FrameBuffer screenSpaceFbBlurPass1;
    Levek::Texture spfBlurVertical;
    
    Levek::FrameBuffer screenSpaceFbBlurPass2;
    Levek::Texture spfBlurHorizontal;
    
    //pass 2
    Levek::FrameBuffer screenSpaceFbPass2;
    Levek::Texture spfOutScene;
    Levek::Texture spfNormal;
    Levek::Texture spfDepth2;

    //thickness
    Levek::FrameBuffer screenSpaceFbThickness;
    Levek::Texture spfThickness1;
    Levek::Texture spfThickness2;
    Levek::Texture spfThickness3;

    Levek::FrameBuffer screenSpaceFbThicknessBlurPass1;
    Levek::FrameBuffer screenSpaceFbThicknessBlurPass2;

    Levek::VertexArray quadVA;
    Levek::Renderer* renderer = nullptr;

    float filterRadius = 8.0f;
    float blurScale = 0.001f;
    float blurDepthFallOff = 0.001f;

    float mBlinnPhongShininess = 2.7f;

    float mThicknessFactor = 0.053f;
    int mThicknessBlurRadius = 5;
    std::vector<float> mNormalDistribution;
    float mThicknessBlurOffset = 4.0f;

    float mVolumetricAbsorption = 1.0f;
    float mDeformationFactor = 0.046f;

    float mDiffuseFactor = 0.2f;
    float mSpecularFactor = 1.0f;

void generateThicknessNormalDistribution() {

    mNormalDistribution.resize(mThicknessBlurRadius);
    for (int i = 0; i < mThicknessBlurRadius; i++) {
        mNormalDistribution[i] = (1.0f / glm::sqrt(2.0f * glm::pi<float>())) * static_cast<float>(std::exp(- static_cast<float>(i*i) / 2.0f));
    }

}

public:

ParticlesRenderingSpf(Levek::RenderingEngine* engine, const std::vector<glm::vec3>& positions, const std::vector<glm::vec4>& colors, ErrorCode& code)
    :
    //pass 1 
    screenSpaceFbPass1(Levek::FrameBuffer(engine->getWindowWidth(), engine->getWindowHeight())),
    spfColor(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::RGBA_FLOAT)),
    spfDepthPass1(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::R_FLOAT)),
    spfDepth(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::DEPTH)),
    
    //blur passes
    screenSpaceFbBlurPass1(Levek::FrameBuffer(engine->getWindowWidth(), engine->getWindowHeight())),
    screenSpaceFbBlurPass2(Levek::FrameBuffer(engine->getWindowWidth(), engine->getWindowHeight())),
    spfDummyDepth(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::DEPTH)),
    spfBlurVertical(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::R_FLOAT)),
    spfBlurHorizontal(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::R_FLOAT)),

    //pass 2
    screenSpaceFbPass2(Levek::FrameBuffer(engine->getWindowWidth(), engine->getWindowHeight())),
    spfOutScene(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::RGBA_FLOAT)),
    spfNormal(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::RGBA_FLOAT)),
    spfDepth2(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::DEPTH)),
    
    //thickness
    screenSpaceFbThickness(Levek::FrameBuffer(engine->getWindowWidth(), engine->getWindowHeight())),
    spfThickness1(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::R_FLOAT)),
    spfThickness2(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::R_FLOAT)),
    spfThickness3(Levek::Texture(engine->getWindowWidth(), engine->getWindowHeight(), Levek::TextureParameters::TextureType::R_FLOAT)),

    screenSpaceFbThicknessBlurPass1 (Levek::FrameBuffer(engine->getWindowWidth(), engine->getWindowHeight())),
    screenSpaceFbThicknessBlurPass2 (Levek::FrameBuffer(engine->getWindowWidth(), engine->getWindowHeight())),

    renderer(engine->getRenderer()),
    resolutionX(engine->getWindowWidth()),
    resolutionY(engine->getWindowHeight()),

    ParticlesPipelineSate(engine, positions, colors, code)
    {
        quadVA.addBuffer(sphereVBO, &sphereLayout);
        std::cout << resolutionX << " " << resolutionY << std::endl;
        //pass 1 fb setup
        screenSpaceFbPass1.addColorAttachment(&spfColor, 0);
        screenSpaceFbPass1.addColorAttachment(&spfDepthPass1, 1);
        screenSpaceFbPass1.addAttachment(&spfDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);

        //blur fb setup
        screenSpaceFbBlurPass1.addColorAttachment(&spfBlurVertical, 0);
        screenSpaceFbBlurPass1.addAttachment(&spfDummyDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);

        screenSpaceFbBlurPass2.addColorAttachment(&spfBlurHorizontal, 0);
        screenSpaceFbBlurPass2.addAttachment(&spfDummyDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);

        //pass 2
        screenSpaceFbPass2.addColorAttachment(&spfOutScene, 0);
        screenSpaceFbPass2.addColorAttachment(&spfNormal, 1);
        screenSpaceFbPass2.addAttachment(&spfDepth2, Levek::FrameBufferProperties::AttachementType::DEPTH);

        //thickness
        screenSpaceFbThickness.addColorAttachment(&spfThickness1, 0);
        screenSpaceFbThickness.addAttachment(&spfDummyDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);

        screenSpaceFbThicknessBlurPass1.addColorAttachment(&spfThickness2, 0);
        screenSpaceFbThicknessBlurPass1.addAttachment(&spfDummyDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);

        screenSpaceFbThicknessBlurPass2.addColorAttachment(&spfThickness3, 0);
        screenSpaceFbThicknessBlurPass2.addAttachment(&spfDummyDepth, Levek::FrameBufferProperties::AttachementType::DEPTH);

        generateThicknessNormalDistribution();
    };
    

    virtual void draw(
        Levek::Texture& backgroundScene,
        Levek::CameraBase& camera,
		const glm::vec3& light_direction,
		float particle_scale
    ) {

        spfColor.clear(glm::vec4(0, 0, 0, 0));
        spfDepthPass1.clear(0.0f);
        spfNormal.clear(glm::vec4(0, 0, 0, 0));
        spfDepth.clear(1.0f);
        spfDepth2.clear(1.0f);
        spfOutScene.clear(glm::vec4(0, 0, 0, 0));
        spfDummyDepth.clear(1.0f);

        //pass 1
        spfShaderPass1.bind();
        spfShaderPass1.setUniformMat4f("vp", camera.getViewProjection());
        spfShaderPass1.setUniformMat4f("p", camera.getProjection());
        spfShaderPass1.setUniformMat4f("view", camera.getView());
        spfShaderPass1.setUniformMat3f("view_inv", camera.getViewInv());
        spfShaderPass1.setUniform3f("light_direction", light_direction);
        spfShaderPass1.setUniform1f("scale", particle_scale);

        renderer->drawInstances(&screenSpaceFbPass1, particlesVA, sphereIBO, &spfShaderPass1, size);

        //pass thickness

        spfDummyDepth.clear(1.0f);
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
	    glBlendFunc(GL_ONE, GL_ONE);
	    glBlendEquation(GL_FUNC_ADD);
	    glDepthMask(GL_FALSE);

        screenSpaceFbThickness.clear();
        //spfThickness.clear(0.0f);

        spfShaderThickness.bind();
        spfShaderThickness.setUniformMat4f("vp", camera.getViewProjection());
        spfShaderThickness.setUniformMat3f("view_inv", camera.getViewInv());
        spfShaderThickness.setUniform1f("scale", particle_scale);
        spfShaderThickness.setUniform1f("factor", mThicknessFactor);

        renderer->drawInstances(&screenSpaceFbThickness, particlesVA, sphereIBO, &spfShaderThickness, size);

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        //gaussian blur thickness
        spfDummyDepth.clear(1.0f);
        screenSpaceFbThicknessBlurPass1.clear();

        spfShaderThicknessBlur.bind();
        spfThickness1.activateAndBind(0);
        spfShaderThicknessBlur.setUniform1i("in_tex", 0);
        spfShaderThicknessBlur.setUniform1i("radius", mThicknessBlurRadius);
        spfShaderThicknessBlur.setUniform1f("weights", mNormalDistribution.data(), mNormalDistribution.size());
        spfShaderThicknessBlur.setUniform2f("direction", glm::vec2{1.0f / spfThickness1.getWidth(), 0.f});
        spfShaderThicknessBlur.setUniform1f("offset", mThicknessBlurOffset);

        renderer->draw(&screenSpaceFbThicknessBlurPass1, &quadVA, sphereIBO, &spfShaderThicknessBlur); 

        spfDummyDepth.clear(1.0f);
        screenSpaceFbThicknessBlurPass2.clear();

        //spfShaderThicknessBlur.bind();
        spfThickness2.activateAndBind(0);
        spfShaderThicknessBlur.setUniform1i("in_tex", 0);
        spfShaderThicknessBlur.setUniform1i("radius", mThicknessBlurRadius);
        spfShaderThicknessBlur.setUniform1f("weights", mNormalDistribution.data(), mNormalDistribution.size());
        spfShaderThicknessBlur.setUniform2f("direction", glm::vec2{0.0f, 1.0f / spfThickness2.getHeight()});
        spfShaderThicknessBlur.setUniform1f("offset", mThicknessBlurOffset);

        renderer->draw(&screenSpaceFbThicknessBlurPass2, &quadVA, sphereIBO, &spfShaderThicknessBlur); 

        //pass blur

        spfBlurVertical.clear(0.0f);
        spfDummyDepth.clear(1.0f);

        spfShaderBlur.bind();
        spfDepthPass1.activateAndBind(0);
        spfShaderBlur.setUniform1i("uTexDepthPass1", 0);
        spfShaderBlur.setUniform2f("uBlurDirection", glm::vec2(0, 1.0f / ((float)resolutionY)));
        spfShaderBlur.setUniform1f("uFilterRadius", filterRadius);
        spfShaderBlur.setUniform1f("uBlurScale", blurScale);
        spfShaderBlur.setUniform1f("uBlurDepthFallOff", blurDepthFallOff);

        renderer->draw(&screenSpaceFbBlurPass1, &quadVA, sphereIBO, &spfShaderBlur); 

        spfBlurHorizontal.clear(0.0f);
        spfDummyDepth.clear(1.0f);

        spfShaderBlur.bind();
        spfBlurVertical.activateAndBind(0);
        spfShaderBlur.setUniform1i("uTexDepthPass1", 0);
        spfShaderBlur.setUniform2f("uBlurDirection", glm::vec2(1.0f / ((float)resolutionX), 0));
        spfShaderBlur.setUniform1f("uFilterRadius", filterRadius);
        spfShaderBlur.setUniform1f("uBlurScale", blurScale);
        spfShaderBlur.setUniform1f("uBlurDepthFallOff", blurDepthFallOff);

        renderer->draw(&screenSpaceFbBlurPass2, &quadVA, sphereIBO, &spfShaderBlur);

        //pass 2

        spfShaderPass2.bind();
        spfBlurHorizontal.activateAndBind(0);
        backgroundScene.activateAndBind(1);
        spfThickness2.activateAndBind(2);
        spfShaderPass2.setUniformMat4f("uMat4P", camera.getProjection());
        glm::mat4 pInv = glm::inverse(camera.getProjection());
        spfShaderPass2.setUniformMat4f("uMat4PInv", pInv);
        glm::vec3 lightDirectionView = glm::normalize(camera.getNormalView() * light_direction);
        spfShaderPass2.setUniform3f("uVec3LightDirectionView", lightDirectionView);
        spfShaderPass2.setUniform1f("uVolumetricAbsorption", mVolumetricAbsorption);
        spfShaderPass2.setUniform1f("uDeformationFactor", mDeformationFactor);
        spfShaderPass2.setUniform1f("uDiffuseFactor", mDiffuseFactor);
        spfShaderPass2.setUniform1f("uSpecularFactor", mSpecularFactor);

        spfShaderPass2.setUniform1i("uTexDepthPass1", 0);
        spfShaderPass2.setUniform1i("uTexBackground", 1);
        spfShaderPass2.setUniform1i("uTexThickness", 2);
        spfShaderPass2.setUniform1f("uShininess", mBlinnPhongShininess);


        renderer->draw(&screenSpaceFbPass2, &quadVA, sphereIBO, &spfShaderPass2);

    }

    void imGui(float scale) {

        ImGui::Text("Fluid");
        ImGui::Text("Color");

        ImGui::SliderFloat("blurRadius", &filterRadius, 1, 50);
        ImGui::SliderFloat("blurScale", &blurScale, 0.00001f, 100.0f);
        ImGui::SliderFloat("blurDepthFallOff", &blurDepthFallOff, 0.001f, 1000.0f);
        ImGui::SliderFloat("shininess", &mBlinnPhongShininess, 0, 40);

    
        ImGui::Text("Normal (from depth)");
        ImGui::SliderFloat("Volumetric absorption", &mVolumetricAbsorption, 0.0f, 10.0f);
        ImGui::SliderFloat("Deformation factor", &mDeformationFactor, 0.0f, 1.0f);

        ImGui::SliderFloat("Diffuse factor", &mDiffuseFactor, 0.0f, 1.0f);
        ImGui::SliderFloat("Specular factor", &mSpecularFactor, 0.0f, 1.0f);

        ImGui::Image((void*)(intptr_t)spfOutScene.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));
        
        ImGui::SliderFloat("ThicknessFactor", &mThicknessFactor, 0.0f, 1.0f);
        
        float saveRadius = mThicknessBlurRadius;
        ImGui::SliderInt("thickness blur radius", &mThicknessBlurRadius, 0, 32);
        if (saveRadius != mThicknessBlurRadius) {
            generateThicknessNormalDistribution();
        }
        
        ImGui::SliderFloat("ThicknessBlurOffset", &mThicknessBlurOffset, 0.0f, 10.0f);
        //ImGui::Image((void*)(intptr_t)spfThickness1.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));
        //ImGui::Image((void*)(intptr_t)spfThickness3.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));
    
    }


};

}