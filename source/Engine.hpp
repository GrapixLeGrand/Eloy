#pragma once

#include <vector>
#include "glm/glm.hpp"
#include "Kernels.hpp"

namespace Eloy {

class Engine {
private:

    int X = 0, Y = 0, Z = 0;
    float particle_radius = static_cast<float>(0.5);
    float particle_diameter = static_cast<float>(2 * particle_radius);
    float kernel_radius = static_cast<float>(3.1f * particle_radius);
    float kernel_factor = static_cast<float>(0.5);

    CubicKernel cubic;
    float cubic_kernel_k;
    float cubic_kernel_l;

    //rest density of fluid
    float rest_density = static_cast<float>(24.0);
    //mass of each particle
    float mass = static_cast<float>(5.0);
    glm::vec3 gravity = glm::vec3(0, -10.0, 0.0);
    float time_step = static_cast<float>(0.01);
    float relaxation_epsilon = static_cast<float>(10.0);

    float s_corr_dq = 0.5f;
    float s_corr_k = 1.0;
    float s_corr_n = 4;

    float c_xsph = 0.1f;
    float epsilon_vorticity = 0.1f;

    std::vector<glm::vec3> velocities;
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> positions_star;
    std::vector<std::vector<int>> neighbors;

    int gridX, gridY, gridZ; //sizes of the grid
    float cell_size; //size of side length of a single grid cell
    int num_grid_cells; //total amount of grid cells

    int num_particles = 0;

public:

Engine();
void step(float dt);

};

}