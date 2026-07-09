#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"

layout (location = 0) out vec4 outSrcColor;
layout (location = 1) out vec4 outBackgroundColor;

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;

layout(set = 0, binding = 17) uniform sampler2D inDepth;

layout(set = 1, binding = 0) uniform sampler2D inColor;

void main()
{
	vec2 texCoord = fs_in.TexCoord;
	outSrcColor = texture(inColor, texCoord);


	float ndcDepth = 	texture(inDepth, fs_in.TexCoord).x;
	vec3 fragPos = EE_DEPTH_TO_WORLD_POS(EE_CAMERA_INDEX, fs_in.TexCoord, ndcDepth);
	vec3 cameraPosition = EE_CAMERA_POSITION(EE_CAMERA_INDEX);
	Camera camera = EE_CAMERAS[EE_CAMERA_INDEX];
	vec3 color = EE_SKY_COLOR(fragPos - cameraPosition);
	//color = vec3(1.0) - exp(-color * EE_CAMERAS[EE_CAMERA_INDEX].reserved_2.w);
	color = pow(color, vec3(1.0 / EE_RENDER_INFO.gamma));
	outBackgroundColor = vec4(color, 1.0);
}
