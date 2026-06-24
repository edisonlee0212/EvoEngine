#extension GL_ARB_shading_language_include : enable

#include "GizmosConstants.glsl"
#include "Basic.glsl"

layout(isolines, equal_spacing) in;

layout (location = 0) in TCS_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	vec4 ProfileProperties;
} tcs_in[];

layout (location = 0) out TES_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	vec3 Tangent;
	vec4 ProfileProperties;
} tes_out;

void main(){
	vec3 position, normal, tangent, tempV;
	vec4 profileProperties, tempP;
	float thickness, tempF;
	EE_SPLINE_INTERPOLATION(tcs_in[0].FragPos, tcs_in[1].FragPos, tcs_in[2].FragPos, tcs_in[3].FragPos, position, tangent, gl_TessCoord.x);
	EE_SPLINE_INTERPOLATION(tcs_in[0].Normal, tcs_in[1].Normal, tcs_in[2].Normal, tcs_in[3].Normal, normal, tempV, gl_TessCoord.x);
	EE_SPLINE_INTERPOLATION(tcs_in[0].Thickness, tcs_in[1].Thickness, tcs_in[2].Thickness, tcs_in[3].Thickness, thickness, tempF, gl_TessCoord.x);
	EE_SPLINE_INTERPOLATION(tcs_in[0].ProfileProperties, tcs_in[1].ProfileProperties, tcs_in[2].ProfileProperties, tcs_in[3].ProfileProperties, profileProperties, tempP, gl_TessCoord.x);

	tes_out.FragPos = position;
	tes_out.Normal = normal;
	tes_out.Thickness = thickness;
	tes_out.Tangent = tangent;
	tes_out.ProfileProperties = profileProperties;
}
