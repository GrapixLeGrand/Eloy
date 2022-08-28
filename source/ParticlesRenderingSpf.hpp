#pragma once

#include "ParticlesRendering.hpp"

namespace Eloy {

class ParticlesRenderingSpf : public ParticlesPipelineSate {

    Levek::Shader spfShader = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/SphereInstanced.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/ScreenSpaceFluid.frag"
    );

public:

ParticlesRenderingSpf(Levek::RenderingEngine* engine, const std::vector<glm::vec3>& positions, const std::vector<glm::vec4>& colors, ErrorCode& code)
    : ParticlesPipelineSate(engine, positions, colors, code) {};


    virtual void draw(Levek::FrameBuffer* fb, Levek::Renderer* renderer) {
		renderer->drawInstances(fb, particlesVA, sphereIBO, &spfShader, size);
	}

	virtual void setUniforms(
		const glm::mat4& vp,
		const glm::mat4& p,
		const glm::mat4& v,
		const glm::mat4& v_inv,
		const glm::vec3& light_direction,
		float particle_scale 
	) {
		spfShader.bind();
        spfShader.setUniformMat4f("vp", vp);
        spfShader.setUniformMat4f("p", p);
        spfShader.setUniformMat4f("view", v);
        spfShader.setUniformMat3f("view_inv", v_inv);
        spfShader.setUniform3f("light_direction", light_direction);
        spfShader.setUniform1f("scale", particle_scale);
	}

};

}