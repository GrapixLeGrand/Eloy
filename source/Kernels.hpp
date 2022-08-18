#pragma once

#include "glm/glm.hpp"

namespace Eloy {

class IKernel {
public:
    virtual ~IKernel() = 0;
    virtual inline float apply(float r) = 0;
};


class IKernelGrad {
public:
    virtual ~IKernelGrad() = 0;
    virtual inline glm::vec3 apply(glm::vec3& r) = 0;
};


class CubicKernel : public IKernel, IKernelGrad {
    float factor = 0;
    float radius = 0;
public:
    CubicKernel(float factor_arg, float radius_arg): factor(factor_arg), radius(radius_arg) {} 
    
    virtual inline float apply(float r) {
        return 0.0f;
    }

    virtual inline glm::vec3 apply(glm::vec3& r) {
        return glm::vec3(0.0);
    }
};

}