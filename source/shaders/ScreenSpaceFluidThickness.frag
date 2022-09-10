#version 430

//a tutorial https://paroj.github.io/gltut/Illumination/Tutorial%2013.html

layout (location = 0) out float thickness;

in vec2 v2f_uv;

uniform float factor;

void main()
{   

    vec2 uv = 2.0 * (v2f_uv - vec2(0.5, 0.5));
    float len = dot(uv, uv);

    if (len <= 1.0) {
        
        //uv.y *= -1;
        vec3 normal = vec3(uv, sqrt(1.0 - len)); //in camera position
        //vec3 position = normalize(normal) * radius + v2f_position_view;
        //vec4 clipPos = p * vec4(position, 1.0);

        thickness = normal.z * factor; //0.005; //normal.z * 0.005;
        //thickness.a = 1.0;
        
    } else {
        discard;
    }

    
}