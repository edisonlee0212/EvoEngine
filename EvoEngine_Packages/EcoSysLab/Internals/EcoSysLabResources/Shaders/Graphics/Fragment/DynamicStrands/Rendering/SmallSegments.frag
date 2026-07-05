#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

#include "DynamicStrandsSmallSegmentsRenderingConstants.glsl"

#include "PerFrame.glsl"
#include "GltfRasterMaterial.glsl"
#define EE_PER_GROUP_SET 2
#include "Lighting.glsl"

layout(location = 0) in VS_OUT {
  vec3 FragPos;
  vec3 Normal;
  vec3 Tangent;
  vec2 TexCoord;
  vec4 Color;
}
fs_in;

layout(location = 0) out vec4 outNormal;
layout(location = 1) out vec4 outMaterial;

void main() {
  Instance instance = EE_INSTANCES[EE_INSTANCE_INDEX];
  vec2 tex_coord = fs_in.TexCoord;
  GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(uint(instance.material_index), tex_coord, tex_coord);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;

  vec3 normal =
      EE_EVALUATE_GLTF_RASTER_NORMAL(uint(instance.material_index), tex_coord, tex_coord, fs_in.Normal, fs_in.Tangent);

  // also store the per-fragment normals into the gbuffer
  outNormal.rgb = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);
  outNormal.a = EE_INSTANCE_INDEX;

  int material_index = instance.material_index;
  if (fs_in.Color.a > 0.7f) {
    material_index = splinter_material_index;
  }
  if (fs_in.Color.a > 0.9f) {
    outMaterial = vec4(fs_in.Color.xyz, instance.info_index + 2);
  } else {
    outMaterial = vec4(tex_coord.x, tex_coord.y, material_index, instance.info_index);
  }
}
