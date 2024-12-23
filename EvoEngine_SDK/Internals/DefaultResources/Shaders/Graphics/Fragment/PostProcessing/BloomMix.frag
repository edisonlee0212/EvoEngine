#extension GL_ARB_shading_language_include : enable

layout (location = 0) out vec4 FragColor;
layout (location = 0) in VS_OUT {
    vec2 TexCoord;
} fs_in;

layout(set = 1, binding = 0) uniform sampler2D originalColor;
layout(set = 1, binding = 1) uniform sampler2D bloomColor;

void main()
{
	vec2 texCoord = fs_in.TexCoord;
	vec4 color = texture(originalColor, texCoord);
	FragColor = vec4(clamp(color.xyz, vec3(0.0f), vec3(1.0f)) + texture(bloomColor, texCoord).xyz, color.w);
}