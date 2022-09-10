#version 430 core
//Quad vert
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoords;
layout (location = 2) in vec3 vertex_normal;

out vec2 v2fUv;

void main()
{   
    vec2 pos = 2.0 * aPos.xy;
    gl_Position = vec4(pos.x, pos.y, 0.0, 1.0);
    v2fUv = vec2(aTexCoords.x, 1.0 - aTexCoords.y);
}