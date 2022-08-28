#pragma once

#include "ParticlesRendering.hpp"

namespace Eloy {

class ParticlesRenderingSpf : public ParticlesPipelineSate {

    Levek::Shader spfShaderPass1 = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass1.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass1.frag"
    );

    Levek::Shader spfShaderPass2 = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass2.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass2.frag"
    );

    /*Levek::VertexBuffer* quadPositionsVBO = nullptr;
    Levek::IndexBuffer* quadIBO = nullptr;
    Levek::VertexBufferLayout quadLayout = Levek::VertexBufferLayout();*/
    Levek::VertexArray quadVA;

public:

ParticlesRenderingSpf(Levek::RenderingEngine* engine, const std::vector<glm::vec3>& positions, const std::vector<glm::vec4>& colors, ErrorCode& code)
    : ParticlesPipelineSate(engine, positions, colors, code) {
        quadVA.addBuffer(sphereVBO, &sphereLayout);
    };


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


    virtual void setUniformsPass2(
		const glm::mat4& p,
		const glm::vec3& light_direction,
        const Levek::Texture& depthTexturePass1,
        const Levek::Texture& background
	) {
        glm::mat4 p_inv = glm::inverse(p);
		spfShaderPass2.bind();
        depthTexturePass1.activateAndBind(0);
        background.activateAndBind(1);
        spfShaderPass2.setUniformMat4f("p", p);
        spfShaderPass2.setUniformMat4f("p_inv", p);
        spfShaderPass2.setUniform3f("light_direction", light_direction);

        spfShaderPass2.setUniform1i("u_depth_pass1", 0);
        spfShaderPass2.setUniform1i("u_background", 1);
        
	}
};

}