import EvoEngine.GltfMaterial;
import EvoEngine.Textures;

public struct GltfAlphaTexCoords {
  public float2 uv0;
  public float2 uv1;
  public float2 ddx_uv0;
  public float2 ddx_uv1;
  public float2 ddy_uv0;
  public float2 ddy_uv1;
};

public GltfAlphaTexCoords EE_GLTF_MAKE_ALPHA_TEX_COORDS(float2 uv0, float2 uv1, float2 ddx_uv0, float2 ddx_uv1,
                                                        float2 ddy_uv0, float2 ddy_uv1) {
  return GltfAlphaTexCoords(uv0, uv1, ddx_uv0, ddx_uv1, ddy_uv0, ddy_uv1);
}

public float EE_GLTF_ALPHA_MASK_BASE_COLOR_ALPHA(uint material_index, GltfAlphaTexCoords tex_coords,
                                                 float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  float alpha = material.pbr_base_color_factor.a * vertex_alpha;
  if (uint(material.pbr_base_color_texture) == 0u) {
    return alpha;
  }
  const GltfTextureInfo texture_info = EE_GLTF_TEXTURE_INFOS[uint(material.pbr_base_color_texture)];
  if (texture_info.index < 0) {
    return alpha;
  }
  float2 uv = texture_info.tex_coord == 1 ? tex_coords.uv1 : tex_coords.uv0;
  float2 ddx_uv = texture_info.tex_coord == 1 ? tex_coords.ddx_uv1 : tex_coords.ddx_uv0;
  float2 ddy_uv = texture_info.tex_coord == 1 ? tex_coords.ddy_uv1 : tex_coords.ddy_uv0;
  uv = EE_GLTF_TRANSFORM_UV(texture_info, float3(uv, 1.0f));
  ddx_uv = EE_GLTF_TRANSFORM_UV(texture_info, float3(ddx_uv, 0.0f));
  ddy_uv = EE_GLTF_TRANSFORM_UV(texture_info, float3(ddy_uv, 0.0f));
  return alpha * EE_TEXTURE_2DS[NonUniformResourceIndex(texture_info.index)].SampleGrad(uv, ddx_uv, ddy_uv).a;
}

public bool EE_GLTF_ALPHA_MASK_PASSES(uint material_index, GltfAlphaTexCoords tex_coords, float vertex_alpha) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  return EE_GLTF_ALPHA_MASK_BASE_COLOR_ALPHA(material_index, tex_coords, vertex_alpha) >= material.alpha_cutoff;
}
