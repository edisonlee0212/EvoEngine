#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout(lines, invocations = 1) in;
layout(triangle_strip, max_vertices = 68) out;

layout (location = 0) in TES_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	vec3 Tangent;
	float TexCoord;
} tes_in[];

layout (location = 0) out VS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
} gs_out;

const float PI2 = 6.28318531;

layout(location = 5) in uint currentInstanceIndexIn[];
layout(location = 5) out uint currentInstanceIndexOut;

void main(){
	
	mat4 cameraProjectionView = EE_CAMERAS[EE_CAMERA_INDEX].projection_view;
	uint instanceIndex = currentInstanceIndexIn[0];
	mat4 model = EE_INSTANCES[instanceIndex].model;
	mat4 inverseModel = inverse(model);
	
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

		//Computing
		vec3 v11 = normalize(vS);        
		vec3 v12 = normalize(cross(vS, tS));
	 
		vec3 v21 = normalize(vT);
		vec3 v22 = normalize(cross(vT, tT)); 

		int ringAmountS = EE_STRANDS_RING_SUBDIVISION(model, worldPosS, modelPosS, thickS);
		int ringAmountT = EE_STRANDS_RING_SUBDIVISION(model, worldPosT, modelPosT, thickT);
		int maxRingAmount = max(ringAmountS, ringAmountT);
		for(int k = 0; k <= maxRingAmount; k += 1)
		{
			
			int tempIS = int(k * ringAmountS / maxRingAmount);
			float angleS = PI2 / ringAmountS * tempIS;

			int tempIT = int(k * ringAmountT / maxRingAmount);
			float angleT = PI2 / ringAmountT * tempIT;

			vec3 newPS = vec3(model * vec4(modelPosS.xyz + (v11 * sin(-angleS) + v12 * cos(-angleS)) * thickS, 1.0));
			vec3 newPT = vec3(model * vec4(modelPosT.xyz + (v21 * sin(-angleT) + v22 * cos(-angleT)) * thickT, 1.0));

			//Source Vertex
			currentInstanceIndexOut = instanceIndex;
			gs_out.FragPos = newPS;
			gs_out.Normal = normalize(newPS - worldPosS);
			gs_out.Tangent = tS;
			gs_out.TexCoord = vec2(1.0 * tempIS / ringAmountS, tes_in[i].TexCoord);
			gl_Position = cameraProjectionView * vec4(newPS, 1);
			EmitVertex();

			//Target Vertex
			currentInstanceIndexOut = instanceIndex;
			gs_out.FragPos = newPT;
			gs_out.Normal = normalize(newPT - worldPosT);
			gs_out.Tangent = tT;
			gs_out.TexCoord = vec2(1.0 * tempIT / ringAmountT, tes_in[i + 1].TexCoord);
			gl_Position = cameraProjectionView * vec4(newPT, 1);
			EmitVertex();
		}
	}

	EndPrimitive();
}