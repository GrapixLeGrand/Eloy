#pragma once

#include "glm/glm.hpp"

namespace Eloy {

class FluidBehavior {
private:
    glm::vec2 mAnalog1;
    glm::vec2 mAnalog2;
public:

    void setAnalog(int index, glm::vec2 newValue) {
        if (index == 0) {
            mAnalog1 = newValue;
        } else if (index == 1) {
            mAnalog2 = newValue;
        }
    }

};

}