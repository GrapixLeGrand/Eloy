#version 430


layout (location = 0) out vec4 out_scene;
layout (location = 1) out vec4 out_normal;

uniform mat4 uMat4P;
uniform mat4 uMat4PInv;
uniform vec3 uVec3LightDirectionView;
uniform float uShininess;

uniform sampler2D uTexDepthPass1;
uniform sampler2D uTexBackground;

in vec2 v2fUv;

//https://github.com/JAGJ10/PositionBasedFluids/blob/master/PositionBasedFluids/fluidFinal.frag
//https://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf
vec3 uvToEye(vec2 p, float z) {
	vec2 pos = p * 2.0 - 1.0;
	vec4 clipPos = vec4(pos, z, 1.0);
	vec4 viewPos = uMat4PInv * clipPos;
	return viewPos.xyz / viewPos.w;
}

void main()
{   

    float depthEye = texture(uTexDepthPass1, v2fUv).r;
    if (depthEye < 0.01) {
        discard;
        return;
    }

    vec3 posEye = uvToEye(v2fUv, depthEye);

    vec2 texOffset = 1.0 / textureSize(uTexDepthPass1, 0);

    //https://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf
    vec3 tmpx = uvToEye(v2fUv + vec2(texOffset.x, 0.0), texture(uTexDepthPass1, v2fUv + vec2(texOffset.x, 0.0)).r);
    vec3 tmpx2 = uvToEye(v2fUv - vec2(texOffset.x, 0.0), texture(uTexDepthPass1, v2fUv - vec2(texOffset.x, 0.0)).r);
    vec3 ddx = tmpx - posEye;
    vec3 ddx2 = posEye - tmpx2;

    if (abs(ddx.z) > abs(ddx2.z)) {
        ddx = ddx2;
    }

    vec3 tmpy = uvToEye(v2fUv + vec2(0.0, texOffset.y), texture(uTexDepthPass1, v2fUv + vec2(0.0, texOffset.y)).r);
    vec3 tmpy2 = uvToEye(v2fUv - vec2(0.0, texOffset.y), texture(uTexDepthPass1, v2fUv - vec2(0.0, texOffset.y)).r);
    vec3 ddy = tmpy - posEye;
    vec3 ddy2 = posEye - tmpy2;
    if (abs(ddy.z) > abs(ddy2.z)) {
        ddy = ddy2;
    }

    vec3 n = -cross(ddx, ddy);
    float l = length(n);
    if (l > 0.0) {
        n /= l;
    }
    //n = normalize(n);

    float diff = max(dot(n, uVec3LightDirectionView), 0.0);
    float specular = 0.0;
    if (diff > 0.0) {
        vec3 viewDirection = normalize(-posEye);
        vec3 halfDirection = normalize(uVec3LightDirectionView + viewDirection);
        float specularAngle = max(dot(halfDirection, n), 0.0);
        specular = pow(specularAngle, 2.0);
    }

    vec3 color = vec3(0.1, 0.7, 0.9);
    out_scene = vec4(color * vec3(0.5) + diff * color + specular * uShininess * color, 1.0);

    
    out_normal = vec4(n, 1.0);
    
}