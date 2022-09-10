#version 430 core

layout (location = 0) out vec4 out_blurred_thickness;
in vec2 v2fUv;

uniform sampler2D in_tex;
uniform int radius;
uniform float weights[32];
uniform vec2 direction;

void main()
{
    float result = texture(in_tex, v2fUv).r * weights[0];
    for (int i = 1; i < radius; i++) {
        result += texture(in_tex, v2fUv + direction * vec2(i)).r * weights[i];
        result += texture(in_tex, v2fUv - direction * vec2(i)).r * weights[i];
    }
    out_blurred_thickness.r = result;
    out_blurred_thickness.a = 1.0;
}