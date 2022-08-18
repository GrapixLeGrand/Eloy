#include "Eloy.hpp"


namespace Eloy {    

Engine::Engine() {

}

void Engine::step(float dt) {

    dt = glm::clamp(dt, 0.001f, 0.01f);

    //integration
    for (int i = 0; i < num_particles; i++) {
        velocities[i] += gravity * mass * dt;
        positions_star[i] = positions[i] + velocities[i] * dt; //prediction
    }

    //find_neighbors_uniform_grid(simulation);

    //solve pressure
    for (int i = 0; i < num_particles; i++) {

        float densitiy = 0.0;
        for (int j = 0; j < neighbors[i].size(); j++) {
            glm::vec3 ij = positions_star[i] - positions_star[neighbors[i][j]];
            float len = glm::length(ij);
            densitiy += mass * simulation->W(simulation, len);
        }
        densitiy += mass * simulation->W(simulation, 0.0);

        //equation 1
        float constraint_i = (densitiy / simulation->rest_density) - 1.0;
        float constraint_gradient_sum = 0.0;
        glm::vec3 grad_current_p = glm::vec3(0.0);

        //equation 8
        for (int j = 0; j < neighbors[i].size(); j++) {
            glm::vec3 temp = simulation->positions_star[i] - simulation->positions_star[neighbors[i][j]];
            glm::vec3 neighbor_grad = -(simulation->mass / simulation->rest_density) * simulation->gradW(simulation, temp);
            constraint_gradient_sum += glm::dot(neighbor_grad, neighbor_grad);
            grad_current_p -= neighbor_grad;
        }

        constraint_gradient_sum += glm::dot(grad_current_p, grad_current_p);

        lambdas[i] = 0.0;
        if (constraint_gradient_sum > 0.0) {
            lambdas[i] = -constraint_i / (constraint_gradient_sum + simulation->relaxation_epsilon);
        }

    }

    for (int i = simulation->ptr_sand_start; i < simulation->ptr_sand_end; i++) {

        //equation 13 (applying pressure force correction)
        //pressures_forces[i] = glm::vec3(0.0);
        glm::vec3 pressure_force = glm::vec3(0.0);
        for (int j = 0; j < neighbors[i].size(); j++) {
            glm::vec3 ij = simulation->positions_star[i] - simulation->positions_star[neighbors[i][j]];
            pressure_force += (lambdas[i] + lambdas[j] + s_coor(simulation, glm::length(ij))) * simulation->gradW(simulation, ij);
        }

        pressure_force /= simulation->rest_density;

        //update prediction
        simulation->positions_star[i] += pressure_force;

        //update velocity
        simulation->positions_star[i].x = resolve_collision(simulation->positions_star[i].x, simulation->particleRadius, X - simulation->particleRadius);
        simulation->positions_star[i].y = resolve_collision(simulation->positions_star[i].y, simulation->particleRadius, Y - simulation->particleRadius);
        simulation->positions_star[i].z = resolve_collision(simulation->positions_star[i].z, simulation->particleRadius, Z - simulation->particleRadius);

        velocities[i] = (simulation->positions_star[i] - simulation->positions[i]) / simulation->time_step;
        simulation->positions[i] = simulation->positions_star[i];

    }
}

}
