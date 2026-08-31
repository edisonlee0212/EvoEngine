import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.RasterVertex;
import EvoEngine.GltfRasterMaterial;
import EvoEngine.LightingFixedSet3;

[shader("fragment")] float4 main(StandardRasterOutput input, bool is_front_face : SV_IsFrontFace) : SV_Target {
  int indirect_lighting_debug_view = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
  int instance_index = int(input.instance_index);
  Instance instance = EE_INSTANCES[instance_index];
  uint material_index = uint(instance.material_index);
  float facing_sign = (is_front_face ? 1.0f : -1.0f) * input.transform_handedness;
  if (EE_GLTF_MATERIALS[material_index].double_sided == 0 && facing_sign < 0.0f)
    discard;
  const GltfTexCoords tex_coords = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(
      input.tex_coord01.xy, input.tex_coord01.zw, input.tex_coord23.xy, input.tex_coord23.zw, ddx(input.tex_coord01.xy),
      ddx(input.tex_coord01.zw), ddx(input.tex_coord23.xy), ddx(input.tex_coord23.zw), ddy(input.tex_coord01.xy),
      ddy(input.tex_coord01.zw), ddy(input.tex_coord23.xy), ddy(input.tex_coord23.zw));
  GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coords, input.color);
  if (EE_GLTF_RASTER_SHOULD_DISCARD(surface))
    discard;
  if (EE_GLTF_MATERIALS[material_index].unlit != 0) {
    return float4(indirect_lighting_debug_view == 0 ? surface.base_color.rgb : float3(0.0f),
                  indirect_lighting_debug_view == 0 ? EE_GLTF_RASTER_OPACITY(surface) : 1.0f);
  }

  float3 normal =
      EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coords, input.normal, input.tangent, input.tangent_handedness);
  normal = normalize(facing_sign * normal);
  float3 view_direction = normalize(EE_CAMERA_POSITION(EE_BASIC_CONSTANTS.camera_index) - input.world_position);
  float depth = EE_LINEARIZE_DEPTH(EE_BASIC_CONSTANTS.camera_index, input.position.z);
  float3 direct =
      EE_FUNC_CALCULATE_LIGHTS(true, surface.base_color.rgb, 1.0f, depth, normal, view_direction, input.world_position,
                               surface.metallic, surface.roughness, surface.specular_f0, surface.specular_f90);
  float3 ambient = EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT(
      surface.base_color.rgb, normal, view_direction, input.world_position, surface.metallic, surface.roughness,
      surface.specular_f0, surface.specular_f90, surface.occlusion, 1.0f);
  if (indirect_lighting_debug_view != 0)
    return float4(ambient, 1.0f);
  float3 emission = EE_GLTF_RASTER_COATED_EMISSION(material_index, surface, tex_coords, input.normal, input.tangent,
                                                   input.tangent_handedness, facing_sign, view_direction);
  return float4(direct + emission + ambient, EE_GLTF_RASTER_OPACITY(surface));
}
