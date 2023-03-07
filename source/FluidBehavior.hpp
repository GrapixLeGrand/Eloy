#pragma once

#include "glm/glm.hpp"

namespace Eloy {

class FluidBehavior {
private:
    glm::vec2 mAnalog1;
    glm::vec2 mAnalog2;
    float mMagnitude = 0.0f;
public:

    void imGui() {
        ImGui::Begin("Fluid behavior");
        ImGui::SliderFloat("magnitude", &mMagnitude, 0.f, 50000.0f);
        ImGui::SliderFloat2("axis0", &mAnalog1[0], -1.0f, 1.0f);
        ImGui::SliderFloat2("axis1", &mAnalog2[0], -1.0f, 1.0f);
        ImGui::End();
    }

    InjectionFunction getFun() {
        std::function<void(InjectionParameters&)> result(
            [&](InjectionParameters& params)->void { 
                
                glm::vec3 gravityCenter {0.f, 0.f, 0.f};
                for (const auto& p : params.positions) { gravityCenter += p; }
                gravityCenter /= static_cast<float>(params.positions.size());

                glm::vec3 direction {mAnalog1.x, mAnalog2.x, mAnalog1.y};
                direction = glm::normalize(direction);
                glm::vec3 target = gravityCenter + mMagnitude * direction;

                /*if (glm::dot(direction, direction) < glm::epsilon<float>()) {
                    return;
                }*/
                std::cout << "hello " << mAnalog1[0] << std::endl;
                for (int i = 0; i < params.positions.size(); i++) {
                    glm::vec3 particleDirection = target - params.positions[i];
                    particleDirection = glm::normalize(particleDirection);
                    params.velocitie[i] +=  mMagnitude * direction * params.parameters.mTimeStep;
                }

            }
        );
        return result;
    }

    void setAnalog(int index, glm::vec2 newValue) {
        if (index == 0) {
            mAnalog1 = newValue;
        } else if (index == 1) {
            mAnalog2 = newValue;
        }
    }

};

}