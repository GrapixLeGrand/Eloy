#version 430 core

layout (location = 0) out vec4 outBlurredDepth;
in vec2 v2fUv;

uniform sampler2D uTexDepthPass1;
uniform float uFilterRadius;
uniform vec2 uBlurDirection;
uniform float uBlurScale;
uniform float uBlurDepthFallOff;

void main() {

    float depth = texture(uTexDepthPass1, v2fUv).r;

    if (depth < 0.01) { //WARNING this parameter seems to change the look of the fluid
        discard;
        return;
    }

    float sum = 0.0;
    float wsum = 0.0;
    //float counter = 0.0;

    //vec2 texelSize = 1.0 / textureSize(uTexDepthPass1, 0);

    for (float x = -uFilterRadius; x <= uFilterRadius; x += 1.0) {
        float s = texture(uTexDepthPass1, v2fUv + x * uBlurDirection).r;
        // spatial domain
        float r = x * uBlurScale;
        float w = exp(-r*r);
        // range domain
        float r2 = (s - depth) * uBlurDepthFallOff;
        float g = exp(-r2*r2);
        sum += s * w * g;
        wsum += w * g;
        //counter += (1.0 / (2.0 * uFilterRadius));
    }

    if (wsum > 0.0) {
        sum /= wsum;
    }

    outBlurredDepth.r = sum;
    outBlurredDepth.a = 1.0;

}

