#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.glsl"
#include "Basic.glsl"
#include "GltfRasterMaterial.glsl"

layout (location = 0) in VS_OUT {
	vec2 TexCoord;
} fs_in;

layout(location = 5) in flat uint currentInstanceIndex;

void main()
{
	uint instanceIndex = currentInstanceIndex;
	uint material_index = uint(EE_INSTANCES[instanceIndex].material_index);
	vec2 tex_coord = fs_in.TexCoord;
	GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord, tex_coord);
	if (EE_GLTF_RASTER_SHOULD_DISCARD(surface)) discard;
}
