import EvoEngine.PerFrame;
import EcoSysLab.DynamicStrandsSmallSegmentsRenderingConstants;
import EvoEngine.GltfBindlessMaterial;

struct EcoSysLabRasterInput {
  [[vk::location(0)]] float3 FragPos : TEXCOORD0;
  [[vk::location(1)]] float3 Normal : TEXCOORD1;
  [[vk::location(2)]] float3 Tangent : TEXCOORD2;
  [[vk::location(3)]] float2 TexCoord : TEXCOORD3;
  [[vk::location(4)]] float4 Color : TEXCOORD4;
};

struct EcoSysLabFragmentOutput {
  [[vk::location(0)]] float4 outGBufferBaseColorAO : SV_Target0;
  [[vk::location(1)]] float4 outGBufferNormalRoughness : SV_Target1;
  [[vk::location(2)]] float4 outGBufferPbrFlags : SV_Target2;
  [[vk::location(3)]] float4 outGBufferEmissive : SV_Target3;
  [[vk::location(4)]] float4 outGBufferUtility : SV_Target4;
};
[shader("fragment")]
EcoSysLabFragmentOutput main(EcoSysLabRasterInput fs_in, bool is_front_face : SV_IsFrontFace) {
  EcoSysLabFragmentOutput output;
  Instance instance = EE_INSTANCES[EE_INSTANCE_INDEX];
  float2 tex_coord = fs_in.TexCoord;
  int material_index = instance.material_index;
  if (fs_in.Color.a > 0.7f) {
    material_index = splinter_material_index;
  }
  float tex_grad = max(length(ddx(tex_coord)), length(ddy(tex_coord)));
  GltfRasterMaterial surface =
      EE_EVALUATE_GLTF_RASTER_SURFACE(uint(material_index), tex_coord, tex_coord, tex_grad);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;

  float3 normal = EE_EVALUATE_GLTF_RASTER_NORMAL(uint(material_index), tex_coord, tex_coord, fs_in.Normal, fs_in.Tangent,
                                                 1.0, tex_grad);

  float facing_sign = (is_front_face ? 1.0 : -1.0) * EE_TRANSFORM_HANDEDNESS(instance.model);
  float3 world_normal = normalize(facing_sign * normal);
  float3 base_color = max(surface.base_color.rgb, float3(0.0));
  float info_index = float(instance.info_index);
  if (fs_in.Color.a > 0.9f) {
    base_color = max(fs_in.Color.rgb, float3(0.0));
    info_index = float(instance.info_index + 2);
  }
  surface.specular_f0 = EE_GLTF_RASTER_REBASE_SPECULAR_F0(uint(material_index), surface, base_color);
  output.outGBufferBaseColorAO = float4(base_color, max(surface.occlusion, 0.0));
  output.outGBufferNormalRoughness = float4(world_normal, surface.roughness);
  output.outGBufferPbrFlags = float4(surface.metallic, surface.specular_f0);
  float encoded_specular_f90 = EE_GLTF_MATERIALS[material_index].unlit != 0 ? -1.0 : surface.specular_f90;
  float3 view_direction = normalize(EE_CAMERA_POSITION(EE_CAMERA_INDEX) - fs_in.FragPos);
  float3 coated_emissive = EE_GLTF_RASTER_COATED_EMISSION(
      uint(material_index), surface, tex_coord, tex_coord, fs_in.Normal, fs_in.Tangent, 1.0, facing_sign,
      view_direction);
  output.outGBufferEmissive = float4(coated_emissive, encoded_specular_f90);
  output.outGBufferUtility = float4(float(EE_INSTANCE_INDEX), info_index, float(material_index), 0.0);
  return output;
}
