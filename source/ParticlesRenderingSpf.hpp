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

    Levek::VertexArray quadVA;
    Levek::Renderer* renderer = nullptr;

    float filterRadius = 8.0f;
    float blurScale = 0.001f;
    float blurDepthFallOff = 0.001f;

    float mBlinnPhongShininess = 2.7f;



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
        spfShaderPass2.setUniformMat4f("uMat4P", camera.getProjection());
        glm::mat4 pInv = glm::inverse(camera.getProjection());
        spfShaderPass2.setUniformMat4f("uMat4PInv", pInv);
        glm::vec3 lightDirectionView = glm::normalize(camera.getNormalView() * light_direction);
        spfShaderPass2.setUniform3f("uVec3LightDirectionView", lightDirectionView);

        spfShaderPass2.setUniform1i("uTexDepthPass1", 0);
        spfShaderPass2.setUniform1i("uTexBackground", 1);
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
        /*
        ImGui::Image((void*)(intptr_t)spfColor.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));
        

        ImGui::Text("depth view");
        ImGui::Image((void*)(intptr_t)spfDepthPass1.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));

        ImGui::Text("depth view V");
        ImGui::Image((void*)(intptr_t)spfBlurVertical.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));

        

        ImGui::Text("depth view H");
        ImGui::Image((void*)(intptr_t)spfBlurHorizontal.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));
        //ImGui::Text("blurred depth view");
        */
        ImGui::Text("Normal (from depth)");
        //ImGui::Image((void*)(intptr_t)spfNormal.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));
        
        ImGui::Image((void*)(intptr_t)spfOutScene.getId(), ImVec2(static_cast<float>(resolutionX) * scale, static_cast<float>(resolutionY) * scale), ImVec2(0, 1), ImVec2(1, 0));

    }

/*
    void blurDepth(Levek::FrameBuffer* fb, Levek::Renderer* renderer) {
        renderer->draw(fb, &quadVA, sphereIBO, &spfShaderBlur);
    }

    virtual void drawPass1(Levek::FrameBuffer* fb, Levek::Renderer* renderer) {
		renderer->drawInstances(fb, particlesVA, sphereIBO, &spfShaderPass1, size);
	}

    virtual void drawPass2(Levek::FrameBuffer* fb, Levek::Renderer* renderer) {
		renderer->draw(fb, &quadVA, sphereIBO, &spfShaderPass2);
	}

	virtual void setUniformsPass1(
		const glm::mat4& vp,
		const glm::mat4& p,
		const glm::mat4& v,
		const glm::mat4& v_inv,
		const glm::vec3& light_direction,
		float particle_scale 
	) {
		spfShaderPass1.bind();
        spfShaderPass1.setUniformMat4f("vp", vp);
        spfShaderPass1.setUniformMat4f("p", p);
        spfShaderPass1.setUniformMat4f("view", v);
        spfShaderPass1.setUniformMat3f("view_inv", v_inv);
        spfShaderPass1.setUniform3f("light_direction", light_direction);
        spfShaderPass1.setUniform1f("scale", particle_scale);
	}


    virtual void setUniformsBlur(
        const Levek::Texture& depthTexturePass1,
        glm::vec2 blurDirection,
        float filterRadius,
        float blurScale,
        float blurDepthFallOff
	) {
        spfShaderBlur.bind();
        depthTexturePass1.activateAndBind(0);
        spfShaderBlur.setUniform1i("uTexDepthPass1", 0);
        spfShaderBlur.setUniform2f("uBlurDirection", blurDirection);
        spfShaderBlur.setUniform1f("uFilterRadius", filterRadius);
        spfShaderBlur.setUniform1f("uBlurScale", blurScale);
        spfShaderBlur.setUniform1f("uBlurDepthFallOff", blurDepthFallOff);
	}

    virtual void setUniformsPass2(
        Levek::CameraBase& camera,
		const glm::vec3& light_direction,
        const Levek::Texture& depthTexturePass1,
        const Levek::Texture& background
	) {
        glm::mat4 p = camera.getProjection();
        glm::mat4 p_inv = glm::inverse(p);
        glm::vec3 lightDirectionView = glm::normalize(camera.getNormalView() * light_direction);
		spfShaderPass2.bind();
        depthTexturePass1.activateAndBind(0);
        background.activateAndBind(1);
        spfShaderPass2.setUniformMat4f("uMat4P", p);
        spfShaderPass2.setUniformMat4f("uMat4PInv", p_inv);
        spfShaderPass2.setUniform3f("uVec3LightDirectionView", lightDirectionView);

        spfShaderPass2.setUniform1i("uTexDepthPass1", 0);
        spfShaderPass2.setUniform1i("uTexBackground", 1);
        
	}
    */
};

}