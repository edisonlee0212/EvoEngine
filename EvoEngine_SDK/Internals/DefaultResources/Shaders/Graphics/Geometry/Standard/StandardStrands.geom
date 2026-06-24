#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout(lines, invocations = 1) in;
layout(triangle_strip, max_vertices = 64) out;

layout (location = 0) in TES_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	vec3 Tangent;
	float TexCoord;
	vec4 Color;
	vec4 ProfileProperties;
} tes_in[];

layout (location = 0) out VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
} gs_out;

const float PI2 = 6.28318531;

layout(location = 7) in flat uint currentInstanceIndexIn[];
layout(location = 5) out flat uint currentInstanceIndexOut;

void main(){
	
	mat4 cameraProjectionView = EE_CAMERAS[EE_CAMERA_INDEX].projection_view;
	uint instanceIndex = currentInstanceIndexIn[0];
	mat4 model = EE_INSTANCES[instanceIndex].model;
	mat4 inverseModel = inverse(model);
	bool vertexColorOnly = EE_MATERIAL_PROPERTIES[EE_INSTANCES[instanceIndex].material_index].sss_c.w > 0.5;
	
	for(int i = 0; i < tes_in.length() - 1; ++i)
	{
		//Reading Data
		vec3 worldPosS = tes_in[i].FragPos;
		vec3 worldPosT = tes_in[i + 1].FragPos;

		vec3 modelPosS = vec3(inverseModel * vec4(worldPosS, 1.0));
		vec3 modelPosT = vec3(inverseModel * vec4(worldPosT, 1.0));

		vec3 vS = tes_in[i].Normal;
		vec3 vT = tes_in[i + 1].Normal;
		
		vec3 tS = tes_in[i].Tangent;
		vec3 tT = tes_in[i + 1].Tangent;

		float thickS = tes_in[i].Thickness;
		float thickT = tes_in[i + 1].Thickness;
		vec4 profileS = tes_in[i].ProfileProperties;
		vec4 profileT = tes_in[i + 1].ProfileProperties;
		//Computing
		vec3 v11 = EE_SAFE_NORMALIZE(vS, vec3(0.0, 1.0, 0.0));
		vec3 v12 = EE_STRANDS_SIDE_VECTOR(v11, tS);
	 
		vec3 v21 = EE_SAFE_NORMALIZE(vT, v11);
		vec3 v22 = EE_STRANDS_SIDE_VECTOR(v21, tT);

		float radiusS = EE_STRAND_PROFILE_EFFECTIVE_RADIUS(profileS, thickS);
		float radiusT = EE_STRAND_PROFILE_EFFECTIVE_RADIUS(profileT, thickT);
		int ringAmountS = EE_STRANDS_RING_SUBDIVISION(model, worldPosS, modelPosS, radiusS);
		int ringAmountT = EE_STRANDS_RING_SUBDIVISION(model, worldPosT, modelPosT, radiusT);
		int maxRingAmount = min(max(ringAmountS, ringAmountT), 31);
		for(int k = 0; k <= maxRingAmount; k += 1)
		{
			
			int tempIS = int(k * ringAmountS / maxRingAmount);
			float profileUS = 1.0 * tempIS / ringAmountS;

			int tempIT = int(k * ringAmountT / maxRingAmount);
			float profileUT = 1.0 * tempIT / ringAmountT;

			vec2 localOffsetS = EE_STRAND_PROFILE_OFFSET(profileS, thickS, profileUS);
			vec2 localOffsetT = EE_STRAND_PROFILE_OFFSET(profileT, thickT, profileUT);
			vec2 localNormalS = EE_STRAND_PROFILE_NORMAL(profileS, profileUS);
			vec2 localNormalT = EE_STRAND_PROFILE_NORMAL(profileT, profileUT);
			vec3 offsetS = v12 * localOffsetS.x + v11 * localOffsetS.y;
			vec3 offsetT = v22 * localOffsetT.x + v21 * localOffsetT.y;
			vec3 profileNormalS = EE_SAFE_NORMALIZE(v12 * localNormalS.x + v11 * localNormalS.y, v11);
			vec3 profileNormalT = EE_SAFE_NORMALIZE(v22 * localNormalT.x + v21 * localNormalT.y, v21);
			vec3 newPS = vec3(model * vec4(modelPosS.xyz + offsetS, 1.0));
			vec3 newPT = vec3(model * vec4(modelPosT.xyz + offsetT, 1.0));

			//Source Vertex
			currentInstanceIndexOut = instanceIndex;
			gs_out.FragPos = newPS;
			gs_out.Normal = profileNormalS;
			gs_out.Tangent = vertexColorOnly ? tes_in[i].Color.rgb : tS;
			gs_out.TexCoord = vec2(profileUS, tes_in[i].TexCoord);
			gl_Position = cameraProjectionView * vec4(newPS, 1);
			EmitVertex();

			//Target Vertex
			currentInstanceIndexOut = instanceIndex;
			gs_out.FragPos = newPT;
			gs_out.Normal = profileNormalT;
			gs_out.Tangent = vertexColorOnly ? tes_in[i + 1].Color.rgb : tT;
			gs_out.TexCoord = vec2(profileUT, tes_in[i + 1].TexCoord);
			gl_Position = cameraProjectionView * vec4(newPT, 1);
			EmitVertex();
		}
	}

	EndPrimitive();
}
