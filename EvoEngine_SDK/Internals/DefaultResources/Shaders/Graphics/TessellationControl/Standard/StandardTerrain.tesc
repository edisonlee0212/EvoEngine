#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout(vertices = 4) out;

layout(location = 0) in VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
} vs_in[];

layout(location = 0) out TCS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
} tcs_out[];

layout(location = 5) in uint currentInstanceIndexIn[];
layout(location = 5) patch out uint currentInstanceIndexOut;

// Compute adaptive tessellation level based on screen-space edge length.
// Uses the same EE_PIXEL_DISTANCE helper that the strand pipeline uses.
float EE_TERRAIN_EDGE_TESS_LEVEL(vec3 worldPosA, vec3 worldPosB) {
	float pixelDistance = EE_PIXEL_DISTANCE(EE_CAMERA_INDEX, worldPosA, worldPosB);
	// Target ~8 pixels per tessellated edge segment. Clamp between 1 and 64 (HW max).
	return clamp(pixelDistance / 8.0, 1.0, 64.0);
}

void main()
{
	currentInstanceIndexOut = currentInstanceIndexIn[gl_InvocationID];

	// Pass through per-vertex data
	tcs_out[gl_InvocationID].FragPos  = vs_in[gl_InvocationID].FragPos;
	tcs_out[gl_InvocationID].Normal   = vs_in[gl_InvocationID].Normal;
	tcs_out[gl_InvocationID].Tangent  = vs_in[gl_InvocationID].Tangent;
	tcs_out[gl_InvocationID].TexCoord = vs_in[gl_InvocationID].TexCoord;

	if (gl_InvocationID == 0) {
		// Quad patch edges: 0-3 bottom, 3-2 right, 2-1 top, 1-0 left
		// (matching Vulkan's quad tessellation winding)
		vec3 p0 = vs_in[0].FragPos;
		vec3 p1 = vs_in[1].FragPos;
		vec3 p2 = vs_in[2].FragPos;
		vec3 p3 = vs_in[3].FragPos;

		// Outer tessellation levels (per edge)
		gl_TessLevelOuter[0] = EE_TERRAIN_EDGE_TESS_LEVEL(p0, p3); // left edge
		gl_TessLevelOuter[1] = EE_TERRAIN_EDGE_TESS_LEVEL(p0, p1); // bottom edge
		gl_TessLevelOuter[2] = EE_TERRAIN_EDGE_TESS_LEVEL(p1, p2); // right edge
		gl_TessLevelOuter[3] = EE_TERRAIN_EDGE_TESS_LEVEL(p3, p2); // top edge

		// Inner tessellation levels (average of opposing edges)
		gl_TessLevelInner[0] = 0.5 * (gl_TessLevelOuter[1] + gl_TessLevelOuter[3]); // horizontal
		gl_TessLevelInner[1] = 0.5 * (gl_TessLevelOuter[0] + gl_TessLevelOuter[2]); // vertical
	}
}
