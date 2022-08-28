#version 430


layout (location = 0) out vec4 out_scene;
layout (location = 1) out vec4 out_normal;

uniform mat4 p;
uniform mat4 p_inv;
uniform vec3 light_direction;

uniform sampler2D u_depth_pass1;
uniform sampler2D u_background;

in vec2 v2f_uv;

void main()
{   

    
    out_scene = vec4(1.0, 1.0, 0.0, 1.0);
    out_normal = vec4(v2f_uv.x, v2f_uv.y, 0, 1.0);
    
}