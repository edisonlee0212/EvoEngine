// Voxel pass adapted from Godot scene_forward_clustered.glsl::MODE_RENDER_SDF at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
import EvoEngine.SdfgiVoxel;
import EvoEngine.GltfBindlessMaterial;
[[vk::push_constant]] ConstantBuffer<SdfgiVoxelPushConstant> draw;
[[vk::binding(0, 1)]] ConstantBuffer<SdfgiVoxelData> voxel;
[[vk::binding(1, 1)]] [vk::image_format("r16ui")] RWTexture3D<uint> albedo;
[[vk::binding(2, 1)]] [vk::image_format("r32ui")] RWTexture3D<uint> emission;
[[vk::binding(3, 1)]] [vk::image_format("r32ui")] RWTexture3D<uint> emission_aniso;
[[vk::binding(4, 1)]] [vk::image_format("r32ui")] RWTexture3D<uint> facing;

[shader("fragment")]
void main(SdfgiVoxelVertex input) {
  GltfShadeMaterial material = EE_GLTF_MATERIALS[draw.material_index];
  GltfTexCoords uv = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(
      input.uv0, input.uv1, ddx(input.uv0), ddx(input.uv1), ddy(input.uv0), ddy(input.uv1));
  float4 base = material.pbr_base_color_factor * saturate(input.color) *
                EE_GLTF_SAMPLE_TEXTURE_SLOT(material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR, uv, float4(1));
  if (material.alpha_mode == EE_GLTF_ALPHA_MODE_MASK && base.a < material.alpha_cutoff)
    discard;
  float3 emitted = material.emissive_factor *
                   EE_GLTF_SAMPLE_TEXTURE_SLOT(material.emissive_texture, EE_GLTF_RASTER_TEXTURE_EMISSIVE, uv, float4(1)).rgb;
  float3 field_position = input.world_position * float3(1, voxel.region_offset_y_mult.w, 1);
  float3 local = (field_position - voxel.cascade_min_cell.xyz) / voxel.cascade_min_cell.w - voxel.region_offset_y_mult.xyz;
  int3 grid = int3(voxel.region_offset_y_mult.xyz) + int3(local);
  uint width, height, depth;
  albedo.GetDimensions(width, height, depth);
  if (any(grid < 0) || any(grid >= int3(width, height, depth)))
    discard;
  float3 normal = normalize(input.normal);
  albedo[grid] = SdfgiPackAlbedo(base.rgb);
  uint ignored;
  InterlockedOr(facing[grid], SdfgiFacingBits(normal), ignored);
  if (length(emitted) > 0.001) {
    uint2 packed = SdfgiPackEmission(emitted, normal);
    emission[grid] = packed.x;
    emission_aniso[grid] = packed.y;
  }
}
