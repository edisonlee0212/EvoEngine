#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout(lines, invocations = 1) in;
layout(triangle_strip, max_vertices = 10) out;

layout (location = 0) in TES_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	vec3 Tangent;
	vec4 ProfileProperties;
} tes_in[];
const float PI2 = 6.28318531;
void main(){
	mat4 model = EE_INSTANCES[EE_INSTANCE_INDEX].model;
	mat4 light_space_matrix = EE_DIRECTIONAL_LIGHTS[EE_CAMERA_INDEX].light_space_matrix[EE_LIGHT_SPLIT_INDEX];
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
		vec4 profileS = tes_in[i].ProfileProperties;
		vec4 profileT = tes_in[i + 1].ProfileProperties;
		
		//Computing
		vec3 v11 = EE_SAFE_NORMALIZE(vS, vec3(0.0, 1.0, 0.0));
		vec3 v12 = EE_STRANDS_SIDE_VECTOR(v11, tS);
	 
		vec3 v21 = EE_SAFE_NORMALIZE(vT, v11);
		vec3 v22 = EE_STRANDS_SIDE_VECTOR(v21, tT);

		int ringSubAmount = 4;

		for(int k = 0; k <= ringSubAmount; k += 1)
		{
			float profileU = 1.0 * k / ringSubAmount;
			vec2 localOffsetS = EE_STRAND_PROFILE_OFFSET(profileS, thickS, profileU);
			vec2 localOffsetT = EE_STRAND_PROFILE_OFFSET(profileT, thickT, profileU);

			vec3 newPS = vec3(model * vec4(modelPosS.xyz + v12 * localOffsetS.x + v11 * localOffsetS.y, 1.0));
			vec3 newPT = vec3(model * vec4(modelPosT.xyz + v22 * localOffsetT.x + v21 * localOffsetT.y, 1.0));

			//Source Vertex
			gl_Position = light_space_matrix * vec4(newPS, 1);
			EmitVertex();

			//Target Vertex
			gl_Position = light_space_matrix * vec4(newPT, 1);
			EmitVertex();
		}
	}

	EndPrimitive();
}
