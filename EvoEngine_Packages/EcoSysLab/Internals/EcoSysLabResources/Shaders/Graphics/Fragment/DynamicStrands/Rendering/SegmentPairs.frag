#extension GL_ARB_shader_draw_parameters : enable
#extension GL_ARB_shading_language_include : enable

layout(push_constant) uniform STRANDS_RENDER_CONSTANTS {
  vec4 color0;
  vec4 color1;

  vec4 position_scale;

  int EE_CAMERA_INDEX;
  uint segment_pairs_size;
  uint color_mode;
  int material_index;
  float multiplier;
  float factor;
};

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

layout(location = 0) out vec4 out_color;

void main() {
  vec2 tex_coord = fs_in.TexCoord;
  GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(uint(material_index), tex_coord, tex_coord);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;

  vec3 normal = EE_EVALUATE_GLTF_RASTER_NORMAL(uint(material_index), tex_coord, tex_coord, fs_in.Normal, fs_in.Tangent);

  // also store the per-fragment normals into the gbuffer
  normal = normalize((gl_FrontFacing ? 1.0 : -1.0) * normal);

  out_color = vec4(1, 0, 0, 0.5);
}
