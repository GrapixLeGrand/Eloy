#version 430 core

layout (location = 0) out vec4 out_blurred_depth;
in vec2 v2fUv;

uniform sampler2D uTexDepthPass1;

#define RADIUS 7

void main()
{
    vec2 texelSize = 1.0 / textureSize(uTexDepthPass1, 0);
    float average = 0.0;
    for (int i = -RADIUS; i <= RADIUS; i++) {
        for (int j = -RADIUS; j <= RADIUS; j++) {
            int ii = 1 * i;
            int jj = 1 * j;
            vec2 offsetIndex = vec2(ii, jj);
            vec2 offset = texelSize * offsetIndex;
            float inside = 1.0; // length(offsetIndex) < RADIUS ? 1.0 : 0.0;
            average += inside * texture(uTexDepthPass1, v2fUv + offset).r;
        }
    }
   average /= RADIUS * RADIUS;
   out_blurred_depth.r = average;
   out_blurred_depth.a = average; //average too?
}