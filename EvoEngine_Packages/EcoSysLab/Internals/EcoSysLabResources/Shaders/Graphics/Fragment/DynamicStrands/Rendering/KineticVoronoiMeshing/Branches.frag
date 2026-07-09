#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_mesh_shader : require

#include "KineticVoronoiRenderingConstants.glsl"

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

layout(location = 5) perprimitiveEXT in FS_PRIM_IN {
  int MaterialIndex;
}
fs_prim;

layout(location = 0) out vec4 outGBufferBaseColorAO;
layout(location = 1) out vec4 outGBufferNormalRoughness;
layout(location = 2) out vec4 outGBufferPbrFlags;
layout(location = 3) out vec4 outGBufferEmissive;
layout(location = 4) out vec4 outGBufferUtility;

void main() {
  Instance instance = EE_INSTANCES[EE_INSTANCE_INDEX];
  int material_index = fs_prim.MaterialIndex;
  if (material_index == -1) {
    material_index = instance.material_index;
  }
  vec2 tex_coord = fs_in.TexCoord;
  GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(uint(material_index), tex_coord, tex_coord);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;

  vec3 normal = EE_EVALUATE_GLTF_RASTER_NORMAL(uint(material_index), tex_coord, tex_coord, fs_in.Normal, fs_in.Tangent);

  vec3 world_normal = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);
  vec3 base_color = max(surface.base_color.rgb, vec3(0.0));
  float info_index = float(instance.info_index);
  if (color_mode != COLOR_STANDARD) {
    base_color = max(fs_in.Color.rgb, vec3(0.0));
    info_index = float(instance.info_index + 2);
  }
  outGBufferBaseColorAO = vec4(base_color, max(surface.occlusion, 0.0));
  outGBufferNormalRoughness = vec4(world_normal, surface.roughness);
  outGBufferPbrFlags = vec4(surface.metallic, 0.0, 0.0, 0.0);
  outGBufferEmissive = vec4(surface.emissive, 0.0);
  outGBufferUtility = vec4(float(EE_INSTANCE_INDEX), info_index, float(material_index), 0.0);
}
