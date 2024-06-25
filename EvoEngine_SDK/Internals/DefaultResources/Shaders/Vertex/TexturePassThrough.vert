layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inTangent;
layout (location = 3) in vec2 inTexCoord;
layout (location = 4) in vec2 inColor;

layout(location = 0) out VS_OUT {
    vec2 TexCoord;
} vs_out;

void main()
{
    vs_out.TexCoord = inTexCoord;
    gl_Position = vec4(inPosition.x, inPosition.y, 0.0, 1.0); 
}