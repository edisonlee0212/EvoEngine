import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.RasterVertex;
import EvoEngine.GltfRasterMaterial;

struct StandardDeferredOutput {
  [[vk::location(0)]] float4 base_color_ao : SV_Target0;
  [[vk::location(1)]] float4 normal_roughness : SV_Target1;
  [[vk::location(2)]] float4 pbr_flags : SV_Target2;
  [[vk::location(3)]] float4 emissive : SV_Target3;
  [[vk::location(4)]] float4 utility : SV_Target4;
};

[shader("fragment")] StandardDeferredOutput main(StandardRasterOutput input, bool is_front_face : SV_IsFrontFace) {
  StandardDeferredOutput output;
  int instance_index = int(input.instance_index);
  Instance instance = EE_INSTANCES[instance_index];
  uint material_index = uint(instance.material_index);
  float facing_sign = (is_front_face ? 1.0f : -1.0f) * input.transform_handedness;
  if (EE_GLTF_MATERIALS[material_index].double_sided == 0 && facing_sign < 0.0f)
    discard;
  const GltfTexCoords tex_coords = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(
      input.tex_coord01.xy, input.tex_coord01.zw, ddx(input.tex_coord01.xy), ddx(input.tex_coord01.zw),
      ddy(input.tex_coord01.xy), ddy(input.tex_coord01.zw));
  GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coords, input.color);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;
  float3 normal =
      EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coords, input.normal, input.tangent, input.tangent_handedness);
  float3 world_normal = normalize(facing_sign * normal);
  float3 view_direction = normalize(EE_CAMERA_POSITION(EE_BASIC_CONSTANTS.camera_index) - input.world_position);
  float3 coated_emissive =
      EE_GLTF_RASTER_COATED_EMISSION(material_index, surface, tex_coords, input.normal, input.tangent,
                                     input.tangent_handedness, facing_sign, view_direction);
  output.base_color_ao = float4(max(surface.base_color.rgb, float3(0.0f)), max(surface.occlusion, 0.0f));
  output.normal_roughness = float4(world_normal, surface.roughness);
  output.pbr_flags = float4(surface.metallic, surface.specular_f0);
  float encoded_specular_f90 = EE_GLTF_MATERIALS[material_index].unlit != 0 ? -1.0f : surface.specular_f90;
  output.emissive = float4(coated_emissive, encoded_specular_f90);
  output.utility = float4(float(instance_index), float(instance.info_index), float(instance.material_index), 0.0f);
  return output;
}
