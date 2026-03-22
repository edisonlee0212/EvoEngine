#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"

layout(quads, equal_spacing, ccw) in;

layout(location = 0) in TCS_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
} tcs_in[];

layout(location = 0) out TES_OUT {
	vec3 FragPos;
	vec3 Normal;
	vec3 Tangent;
	vec2 TexCoord;
} tes_out;

layout(location = 5) patch in uint currentInstanceIndexIn;
layout(location = 5) out flat uint currentInstanceIndexOut;

void main()
{
	currentInstanceIndexOut = currentInstanceIndexIn;

	float u = gl_TessCoord.x;
	float v = gl_TessCoord.y;

	// Bilinear interpolation across the quad patch (4 control points)
	// Patch vertex order: 0=BL, 1=BR, 2=TR, 3=TL
	vec3 pos = mix(mix(tcs_in[0].FragPos,  tcs_in[1].FragPos,  u),
	               mix(tcs_in[3].FragPos,  tcs_in[2].FragPos,  u), v);

	vec3 nrm = mix(mix(tcs_in[0].Normal,   tcs_in[1].Normal,   u),
	               mix(tcs_in[3].Normal,   tcs_in[2].Normal,   u), v);

	vec3 tan = mix(mix(tcs_in[0].Tangent,  tcs_in[1].Tangent,  u),
	               mix(tcs_in[3].Tangent,  tcs_in[2].Tangent,  u), v);

	vec2 uv  = mix(mix(tcs_in[0].TexCoord, tcs_in[1].TexCoord, u),
	               mix(tcs_in[3].TexCoord, tcs_in[2].TexCoord, u), v);

	nrm = normalize(nrm);
	tan = normalize(tan - dot(tan, nrm) * nrm);

	// Sample displacement map. Uses the material's displacement texture index.
	int instance_index = int(currentInstanceIndexIn);
	Instance instance = EE_INSTANCES[instance_index];
	MaterialProperties mat = EE_MATERIAL_PROPERTIES[instance.material_index];

	float displacement = 0.0;
	if (mat.displacement_map_index >= 0) {
		displacement = texture(EE_TEXTURE_2DS[mat.displacement_map_index], uv).r;
	}

	// Apply displacement along vertex normal.
	// displacement_intensity is stored in the material; default scale = 1.0.
	pos += nrm * displacement * mat.displacement_intensity;

	tes_out.FragPos  = pos;
	tes_out.Normal   = nrm;
	tes_out.Tangent  = tan;
	tes_out.TexCoord = uv;

	gl_Position = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(pos, 1.0);
}
