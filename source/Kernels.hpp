#pragma once

#include "glm/glm.hpp"
#include "glm/gtc/constants.hpp"

namespace Eloy {

class IKernel {
public:
    virtual ~IKernel() {};
    virtual inline float W(float r) = 0;
    virtual inline glm::vec3 WGrad(glm::vec3& r) = 0;
};

class CubicKernel : public IKernel {

    
    float mK = static_cast<float>(1);
    float mL = static_cast<float>(1);
public:
    float mFactor = static_cast<float>(1);
    float mRadius = static_cast<float>(0);
    
    CubicKernel()
        :
        mK(static_cast<float>(0)), 
        mL(static_cast<float>(0)),
        mRadius(static_cast<float>(0)),
        mFactor(static_cast<float>(0)) {}

    CubicKernel(float radius, float factor)
        :
        mK(static_cast<float>(0)), 
        mL(static_cast<float>(0)),
        mRadius(radius),
        mFactor(factor) {
        float h3 = std::pow(mRadius, 3);
        mK = static_cast<float>(8) / (glm::pi<float>() * h3);
        mL = static_cast<float>(48) / (glm::pi<float>() * h3);
    }

    virtual inline float W(float r) {
        float q = (r * mFactor) / mRadius;
        float result = 0.0f;
        if (q <= 1.0f) {
            if (q <= 0.5f) {
                float q2 = q * q;
                float q3 = q2 * q;
                result = mK * (6.0f * q3 - 6.0f * q2 + 1.0f);
            } else {
                result = mK * (2.0f * std::pow(1.0f - q, 3.0f));
            }
        }
        return result;
    }

    virtual inline glm::vec3 WGrad(glm::vec3& r) {
        glm::vec3 result = glm::vec3(0.0);
        glm::vec3 rr = r;
        r *= mFactor;
        float rl = glm::length(rr); // * mFactor;
        float q = rl / mRadius;

        if (rl > 1.0e-5f && q <= 1.0f) {
            const glm::vec3 grad_q = (1.0f / (rl * mRadius)) * rr;
            if (q <= 0.5f) {
                result = mL * q * (3.0f * q - 2.0f) * grad_q;
            } else {
                const float f = 1.0f - q;
                result = mL * (- f * f) * grad_q;
            }
        }
        return result;
    }
};

}