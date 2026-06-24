#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout(isolines, equal_spacing) in;

layout (location = 0) in TCS_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	float TexCoord;
	vec4 Color;
	vec4 ProfileProperties;
} tcs_in[];

layout (location = 0) out TES_OUT {
	vec3 FragPos;
	float Thickness;
	vec3 Normal;
	vec3 Tangent;
	float TexCoord;
	vec4 Color;
	vec4 ProfileProperties;
} tes_out;

layout(location = 6) patch in uint currentInstanceIndexIn;
layout(location = 7) out flat uint currentInstanceIndexOut;

void main(){
	currentInstanceIndexOut = currentInstanceIndexIn;
	vec3 position, normal, tangent, tempV;
	vec4 color, profileProperties, tempC;
	float thickness, tex_coord, tempF;
	EE_SPLINE_INTERPOLATION(tcs_in[0].FragPos, tcs_in[1].FragPos, tcs_in[2].FragPos, tcs_in[3].FragPos, position, tangent, gl_TessCoord.x);
	EE_SPLINE_INTERPOLATION(tcs_in[0].Normal, tcs_in[1].Normal, tcs_in[2].Normal, tcs_in[3].Normal, normal, tempV, gl_TessCoord.x);
	EE_SPLINE_INTERPOLATION(tcs_in[0].TexCoord, tcs_in[1].TexCoord, tcs_in[2].TexCoord, tcs_in[3].TexCoord, tex_coord, tempF, gl_TessCoord.x);
	EE_SPLINE_INTERPOLATION(tcs_in[0].Thickness, tcs_in[1].Thickness, tcs_in[2].Thickness, tcs_in[3].Thickness, thickness, tempF, gl_TessCoord.x);
	EE_SPLINE_INTERPOLATION(tcs_in[0].Color, tcs_in[1].Color, tcs_in[2].Color, tcs_in[3].Color, color, tempC, gl_TessCoord.x);
	EE_SPLINE_INTERPOLATION(tcs_in[0].ProfileProperties, tcs_in[1].ProfileProperties, tcs_in[2].ProfileProperties, tcs_in[3].ProfileProperties, profileProperties, tempC, gl_TessCoord.x);

	tes_out.TexCoord = tex_coord;
	tes_out.FragPos = position;
	tes_out.Normal = normal;
	tes_out.Thickness = thickness;
	tes_out.Tangent = tangent;
	tes_out.Color = clamp(color, vec4(0.0), vec4(1.0));
	tes_out.ProfileProperties = profileProperties;
}
