#extension GL_ARB_shading_language_include : enable

#include "SSRConstants.glsl"

layout (location = 0) out vec4 FragColor;
layout (location = 0) in VS_OUT {
    vec2 TexCoord;
} fs_in;

layout(set = 0, binding = 0) uniform sampler2D originalColor;
layout(set = 0, binding = 1) uniform sampler2D reflectedColorVisibility;

void main()
{
    vec3 color = texture(originalColor, fs_in.TexCoord).rgb;
    vec4 reflected = texture(reflectedColorVisibility, fs_in.TexCoord);
    vec3 result = color * (1.0 - reflected.a) + reflected.rgb * reflected.a;
    FragColor = vec4(result, 1.0);
}