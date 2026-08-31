import EvoEngine.GltfBindlessMaterial;

struct PushConstant {
  uint4 material_indices;
};

[[vk::push_constant]] PushConstant push_constant;

[[vk::binding(13, 0)]]
RWStructuredBuffer<float4> values;

[numthreads(2, 1, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const GltfTexCoords tex_coords = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(
      float2(0.1, 0.2), float2(0.2, 0.3), float2(0.3, 0.4), float2(0.4, 0.5), float2(0.01, 0.02), float2(0.03, 0.04),
      float2(0.11, 0.12), float2(0.13, 0.14), float2(0.05, 0.06), float2(0.07, 0.08), float2(0.21, 0.22),
      float2(0.23, 0.24));
  const uint material_index =
      dispatch_thread_id.x == 0u ? push_constant.material_indices.x : push_constant.material_indices.y;
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  values[dispatch_thread_id.x] =
      EE_GLTF_SAMPLE_TEXTURE_SLOT(material.pbr_base_color_texture, 0, tex_coords, float4(0.125, 0.25, 0.5, 1.0));

  if (dispatch_thread_id.x == 0u) {
    const GltfShadeMaterial missing_material = EE_GLTF_MATERIALS[push_constant.material_indices.z];
    values[2] = EE_GLTF_SAMPLE_TEXTURE_SLOT(missing_material.pbr_base_color_texture, 0, tex_coords,
                                            float4(0.125, 0.25, 0.5, 1.0));

    const GltfShadeMaterial pending_material = EE_GLTF_MATERIALS[push_constant.material_indices.w];
    const float4 pending_sample = EE_GLTF_SAMPLE_TEXTURE_SLOT(pending_material.pbr_base_color_texture, 0, tex_coords,
                                                              float4(0.125, 0.25, 0.5, 1.0));
    values[3] = float4(all(!isnan(pending_sample)) && all(!isinf(pending_sample)) ? 1.0 : 0.0);

    const GltfTextureInfo texture_info = EE_GLTF_TEXTURE_INFOS[uint(material.pbr_base_color_texture)];
    float2 ddx_uv;
    float2 ddy_uv;
    EE_GLTF_TEXTURE_GRADIENTS(texture_info, tex_coords, ddx_uv, ddy_uv);
    values[4] = float4(ddx_uv, ddy_uv);
    values[5] = float4(EE_GLTF_TEXTURE_UV(texture_info, tex_coords),
                       EE_GLTF_SELECT_TEX_COORD(texture_info.tex_coord, tex_coords));
  }
}
