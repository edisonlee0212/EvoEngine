#extension GL_ARB_shading_language_include : enable

#include "PerFrame.slangh"
#include "SSRConstants.slangh"

layout (location = 0) out vec4 FragColor;
layout (location = 0) in VS_OUT {
    vec2 TexCoord;
} fs_in;

layout(set = 1, binding = 0) uniform sampler2D originalColor;
layout(set = 1, binding = 1) uniform sampler2D reflectedColorVisibility;

layout(set = 2, binding = 21) uniform sampler2D inNormalRoughness;
layout(set = 2, binding = 22) uniform sampler2D inPbrFlags;

void main()
{
    vec2 texCoord = fs_in.TexCoord;

    float roughness = texture(inNormalRoughness, texCoord).a;
	float metallic = texture(inPbrFlags, texCoord).x;

    vec4 color = texture(originalColor, texCoord);
    vec4 reflected = texture(reflectedColorVisibility, texCoord);

    float factor = clamp(reflected.a * metallic * (1.f - roughness), 0.0f, 1.0f);

    vec3 result = color.xyz * (1.0f - factor) + reflected.rgb * factor;
    FragColor = vec4(result.xyz, color.w);
}
