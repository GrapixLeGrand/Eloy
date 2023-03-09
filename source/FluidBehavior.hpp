#pragma once

#include "Behavior.hpp"
#include "glm/glm.hpp"

namespace Eloy {

class FluidBehavior : public IBehavior {
private:
    glm::vec2 mAnalog1;
    glm::vec2 mAnalog2;
    float mMagnitude = 0.0f;
    float mMagnitudeGravityCenter = 0.0f;

    glm::vec3 mAnalogCurrentDirection {0, 1, 0};

    bool mZeroGravity = false;
    glm::vec3 mSaveGravity {0, 0, 0};

public:

    virtual void start(InjectionParameters& params) {
        std::cout << "start fluid behavior" << std::endl;
        mSaveGravity = params.parameters.mGravity;
    }

    virtual void step(InjectionParameters& params) {
        glm::vec3 gravityCenter {0.f, 0.f, 0.f};
        for (const auto& p : params.positions) { gravityCenter += p; }
        gravityCenter /= static_cast<float>(params.positions.size());

        glm::vec3 direction {mAnalog1.x, mAnalog2.y, mAnalog1.y};

        if (glm::dot(direction, direction) <= glm::epsilon<float>()) {
            std::cout << "joystick is null" << std::endl;
            return;
        }

        direction = glm::normalize(direction);
        mAnalogCurrentDirection = direction;

        glm::vec3 target = gravityCenter + mMagnitudeGravityCenter * direction;

        for (int i = 0; i < params.positions.size(); i++) {
            glm::vec3 particleDirection = target - params.positions[i];
            particleDirection = glm::normalize(particleDirection);
            //params.velocitie[i] +=  mMagnitude * direction * params.parameters.mTimeStep;
            params.positions[i] += particleDirection * mMagnitude * params.parameters.mTimeStep; //* params.positions[i] * 
        }

        if (mZeroGravity) {
            params.gravity = glm::vec3(0.f);
        } else {
            params.gravity = mSaveGravity;
        }
    }

    void imGui() {
        ImGui::Begin("Fluid behavior");
        ImGui::SliderFloat("magnitude", &mMagnitude, 0.f, 2.0f);
        ImGui::SliderFloat("gravity center", &mMagnitudeGravityCenter, 0, 2.f);
        ImGui::SliderFloat2("axis0", &mAnalog1[0], -1.0f, 1.0f);
        ImGui::SliderFloat2("axis1", &mAnalog2[0], -1.0f, 1.0f);

        vgm::Vec3 v;
        v.x = mAnalogCurrentDirection.x;
        v.y = mAnalogCurrentDirection.y;
        v.z = mAnalogCurrentDirection.z;
        ImGui::gizmo3D("##Dir1", v, 100.0f);

        ImGui::Checkbox("zero gravity", &mZeroGravity);

        ImGui::End();
    }

    InjectionFunction getFun() {
        std::function<void(InjectionParameters&)> result(
            [&](InjectionParameters& params)->void { 
                
                glm::vec3 gravityCenter {0.f, 0.f, 0.f};
                for (const auto& p : params.positions) { gravityCenter += p; }
                gravityCenter /= static_cast<float>(params.positions.size());

                glm::vec3 direction {mAnalog1.x, mAnalog2.y, mAnalog1.y};

                if (glm::dot(direction, direction) <= glm::epsilon<float>()) {
                    std::cout << "joystick is null" << std::endl;
                    return;
                }

                direction = glm::normalize(direction);
                mAnalogCurrentDirection = direction;

                glm::vec3 target = gravityCenter + mMagnitudeGravityCenter * direction;

                for (int i = 0; i < params.positions.size(); i++) {
                    glm::vec3 particleDirection = target - params.positions[i];
                    particleDirection = glm::normalize(particleDirection);
                    //params.velocitie[i] +=  mMagnitude * direction * params.parameters.mTimeStep;
                    params.positions[i] += particleDirection * mMagnitude * params.parameters.mTimeStep; //* params.positions[i] * 
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

    void setGravity(bool state) {
        mZeroGravity = state;
        /*if (state && !isButtonBeingPressed) { //rising edge
            isButtonBeingPressed = false;
            mNextGravity = glm::zero;  
        }

        mEnableGravity = state;*/
    }


};

}