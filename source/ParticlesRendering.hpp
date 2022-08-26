#pragma once

#include "LevekGL.hpp"
#include "Error.hpp"

namespace Eloy {

struct ParticlesPipelineSate {

	Levek::Model* billboardModel = nullptr;
	const Levek::Mesh* billboardMesh = nullptr;

	Levek::VertexBuffer* particlesPositionsVBO = nullptr;// = Levek::VertexBuffer((void*) simulation.positions, (size_t) simulation.num_sand_particles * 3 * 4);
    Levek::VertexBuffer* particlesColorsVBO = nullptr;// = Levek::VertexBuffer((void*) simulation.colors, (size_t) simulation.num_sand_particles * 4 * 4);

    Levek::VertexBuffer* sphereVBO = nullptr; // = Levek::VertexBuffer(sphere);
    Levek::IndexBuffer* sphereIBO = nullptr; // = Levek::IndexBuffer(sphere);
    Levek::VertexBufferLayout sphereLayout = Levek::VertexBufferLayout();
    Levek::VertexBufferLayout instanceLayout = Levek::VertexBufferLayout(); 
    Levek::VertexBufferLayout colorLayout = Levek::VertexBufferLayout();
	Levek::VertexArray* particlesVA;
	Levek::Shader shaderInstances = Levek::ShaderFactory::makeFromFile(
        ELOY_SOURCE_DIRECTORY"/shaders/SphereInstanced.vert",
        ELOY_SOURCE_DIRECTORY"/shaders/SphereInstanced.frag"
    );

	size_t size = 0;

	/**
	 * @brief Construct a new Particles Pipeline Sate object
	 * 
	 * @param positions 
	 * @param colors 
	 * @param size in num positions (NOT BYTES!)
	 */
	ParticlesPipelineSate(Levek::RenderingEngine* engine, const std::vector<glm::vec3>& positions, const std::vector<glm::vec4>& colors, ErrorCode& code) {

		
		particlesPositionsVBO = new Levek::VertexBuffer(positions);
		particlesColorsVBO = new Levek::VertexBuffer(colors);
		this->size = positions.size();
        code = ELOY_ERROR_OK;

        if (positions.size() != colors.size()) {
            code = ELOY_ERROR_BAD_SIZE;
        }

		Levek::ModelLoader* meshLoader = engine->getModelLoader();
    	billboardModel = meshLoader->loadFromFile(ELOY_SOURCE_DIRECTORY"/models/plane.obj");
    	assert(billboardModel != nullptr);
		billboardMesh = billboardModel->getMesh(0);
		assert(billboardMesh != nullptr);
		sphereVBO = new Levek::VertexBuffer(billboardMesh);
		sphereIBO = new Levek::IndexBuffer(billboardMesh);

		sphereLayout.push<glm::vec3>(1); //sphere position
    	sphereLayout.push<glm::vec2>(1); //sphere textures
    	sphereLayout.push<glm::vec3>(1); //sphere normal 
    	instanceLayout.push<glm::vec3>(1, 1); //instance offset (per instance)
    	colorLayout.push<glm::vec4>(1, 1);

		particlesVA = new Levek::VertexArray();
    	particlesVA->addBuffer(sphereVBO, &sphereLayout);
    	particlesVA->addBuffer(particlesPositionsVBO, &instanceLayout);
    	particlesVA->addBuffer(particlesColorsVBO, &colorLayout);

	}

	~ParticlesPipelineSate() {
		delete particlesPositionsVBO;
		delete particlesColorsVBO;
		delete billboardModel;
		delete billboardMesh;
		delete sphereVBO;
		delete sphereIBO;
		delete particlesVA;
	}
	
	/**
	 * @brief 
	 * 
	 * @param positions 
	 * @param new_size in num of particles (NOT bytes)
	 */
	void updatePositions(const std::vector<glm::vec3>& positions) {
        size = positions.size();
		particlesPositionsVBO->update(positions);
	}

	/**
	 * @brief 
	 * 
	 * @param positions 
	 * @param new_size in num of particles (NOT bytes)
	 */
	void updateColors(const std::vector<glm::vec4>& colors) {
        size = colors.size();
		particlesColorsVBO->update(colors);
	}

	void draw(Levek::Renderer* renderer) {
		renderer->drawInstances(particlesVA, sphereIBO, &shaderInstances, size);
	}

	void draw(Levek::FrameBuffer* fb, Levek::Renderer* renderer) {
		renderer->drawInstances(fb, particlesVA, sphereIBO, &shaderInstances, size);
	}

	void setUniforms(
		const glm::mat4& vp,
		const glm::mat4& p,
		const glm::mat4& v,
		const glm::mat4& v_inv,
		const glm::vec3& light_direction,
		float particle_scale 
	) {
		shaderInstances.bind();
        shaderInstances.setUniformMat4f("vp", vp);
        shaderInstances.setUniformMat4f("p", p);
        shaderInstances.setUniformMat4f("view", v);
        shaderInstances.setUniformMat3f("view_inv", v_inv);
        shaderInstances.setUniform3f("light_direction", light_direction);
        shaderInstances.setUniform1f("scale", particle_scale);
	}

};

}