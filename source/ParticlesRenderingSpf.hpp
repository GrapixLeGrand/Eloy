#pragma once

#include "ParticlesRendering.hpp"

namespace Eloy {

class ParticlesRenderingSpf : public ParticlesPipelineSate {

    Levek::Shader spfShaderPass1 = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/SphereInstanced.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass1.frag"
    );

    Levek::Shader spfShaderPass2 = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/SphereInstanced.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluidPass2.frag"
    );

public:

ParticlesRenderingSpf(Levek::RenderingEngine* engine, const std::vector<glm::vec3>& positions, const std::vector<glm::vec4>& colors, ErrorCode& code)
    : ParticlesPipelineSate(engine, positions, colors, code) {};


    virtual void drawPass1(Levek::FrameBuffer* fb, Levek::Renderer* renderer) {
		renderer->drawInstances(fb, particlesVA, sphereIBO, &spfShaderPass1, size);
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

};

}