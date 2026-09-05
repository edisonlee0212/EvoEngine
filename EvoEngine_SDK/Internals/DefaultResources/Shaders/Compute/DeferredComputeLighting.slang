import EvoEngine.GltfRasterMaterial;
import EvoEngine.GBuffer;
import EvoEngine.LightingFixedSet3;
#ifdef EE_AUTOMATIC_SDFGI
import EvoEngine.SdfgiLighting;
#endif

[[vk::binding(17, 1)]]
Sampler2D inDepth;

[[vk::binding(4, 3)]]
Sampler2D inAmbientOcclusion;

[[vk::binding(0, 4)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> inOutUvBaseColorAo;
[[vk::binding(1, 4)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> inOutFrameNormalRoughness;
[[vk::binding(2, 4)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> inOutDdxPbrFlags;
[[vk::binding(3, 4)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> inOutDdyEmissive;
[[vk::binding(4, 4)]]
Texture2D<uint4> inMetadata;
[[vk::binding(5, 4)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outColor;

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  inOutUvBaseColorAo.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  if (pixel.x >= width || pixel.y >= height) {
    return;
  }

  const int camera_index = EE_BASIC_CONSTANTS.camera_index;
  const int indirect_lighting_debug_view = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
  const uint sdfgi_debug_view = EE_BASIC_CONSTANTS.meshlet_culling_flags >> 8u;
  const bool scene_camera = (EE_BASIC_CONSTANTS.meshlet_culling_flags & 1u) != 0u;
  const float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(width, height);
  const float depth = inDepth.SampleLevel(tex_coord, 0.0f).x;
  if (!isfinite(depth) || depth >= 1.0f) {
    const float3 world_position = EE_DEPTH_TO_WORLD_POS(camera_index, tex_coord, 1.0f);
    const float3 sky_color = EE_SKY_COLOR(world_position - EE_CAMERA_POSITION(camera_index));
    outColor[pixel] = float4(indirect_lighting_debug_view == 0 && sdfgi_debug_view == 0u ? sky_color : float3(0.0f), 1.0f);
    return;
  }

  const uint4 packed_metadata = inMetadata.Load(int3(pixel, 0));
  const float4 raw_uv = inOutUvBaseColorAo[pixel];
  const float4 raw_frame = inOutFrameNormalRoughness[pixel];
  const float4 raw_ddx = inOutDdxPbrFlags[pixel];
  const float4 raw_ddy = inOutDdyEmissive[pixel];
  const EeGBufferMetadata metadata = EE_GBUFFER_UNPACK_METADATA(packed_metadata);
  const GltfTexCoords tex_coords = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(
      raw_uv.xy, raw_uv.zw, raw_ddx.xy, raw_ddx.zw, raw_ddy.xy, raw_ddy.zw);
  GltfRasterMaterial surface =
      EE_EVALUATE_GLTF_RASTER_SURFACE(metadata.material_index, tex_coords, metadata.vertex_color);

  const float3 geometric_normal = EE_GBUFFER_OCT_DECODE(raw_frame.xy);
  const float3 tangent = EE_GBUFFER_OCT_DECODE(raw_frame.zw);
  const float tangent_handedness = metadata.negative_tangent_handedness ? -1.0f : 1.0f;
  const float3 shading_normal = EE_EVALUATE_GLTF_RASTER_NORMAL(metadata.material_index, tex_coords, geometric_normal,
                                                               tangent, tangent_handedness);
  if (metadata.vertex_color_replaces_base_color) {
    const float3 replacement_base_color = max(metadata.vertex_color.rgb, float3(0.0f));
    surface.specular_f0 =
        EE_GLTF_RASTER_REBASE_SPECULAR_F0(metadata.material_index, surface, replacement_base_color);
    surface.base_color.rgb = replacement_base_color;
  }

  const float3 world_position = EE_DEPTH_TO_WORLD_POS(camera_index, tex_coord, depth);
  const float3 camera_position = EE_CAMERA_POSITION(camera_index);
  const float3 view_direction = normalize(camera_position - world_position);
  const float3 coated_emissive = EE_GLTF_RASTER_COATED_EMISSION(metadata.material_index, surface, tex_coords,
                                                                geometric_normal, tangent, tangent_handedness, 1.0f,
                                                                view_direction);
  const float encoded_specular_f90 =
      EE_GLTF_MATERIALS[metadata.material_index].unlit != 0 ? -1.0f : surface.specular_f90;

  const float3 resolved_base_color = max(surface.base_color.rgb, float3(0.0f));
  const float resolved_material_ao = max(surface.occlusion, 0.0f);
  inOutUvBaseColorAo[pixel] = float4(resolved_base_color, resolved_material_ao);
  inOutFrameNormalRoughness[pixel] = float4(shading_normal, surface.roughness);
  inOutDdxPbrFlags[pixel] = float4(surface.metallic, surface.specular_f0);
  inOutDdyEmissive[pixel] = float4(coated_emissive, encoded_specular_f90);

  if (sdfgi_debug_view >= 7u && sdfgi_debug_view <= 9u) {
    float3 isolated = sdfgi_debug_view == 9u ? float3(1, 0, 0) : float3(0);
#ifdef EE_AUTOMATIC_SDFGI
    if (encoded_specular_f90 >= 0.0f)
      isolated = EE_FUNC_CALCULATE_SDFGI_ENVIRONMENTAL_LIGHT(resolved_base_color, shading_normal,
          view_direction, world_position, surface.metallic, surface.roughness, surface.specular_f0,
          encoded_specular_f90, resolved_material_ao, inAmbientOcclusion.SampleLevel(tex_coord, 0.0f).r);
#endif
    outColor[pixel] = float4(isolated, 1);
    return;
  }

  if (!scene_camera && encoded_specular_f90 < 0.0f) {
    outColor[pixel] = float4(indirect_lighting_debug_view == 0 ? resolved_base_color : float3(0.0f), 1.0f);
    return;
  }

  const float linear_depth = EE_LINEARIZE_DEPTH(camera_index, depth);
  if (indirect_lighting_debug_view == 0) {
    const float4 shadow_debug_color = EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(linear_depth, world_position);
    if (shadow_debug_color.a > 0.0f) {
      outColor[pixel] = shadow_debug_color;
      return;
    }
  }

  float3 albedo = resolved_base_color;
  if (scene_camera) {
    const float3 material_kernel =
        abs(EE_KERNEL_BLOCK.uniform_kernel[metadata.material_index % EE_MAX_KERNEL_AMOUNT].xyz);
    const float3 instance_kernel =
        abs(EE_KERNEL_BLOCK.uniform_kernel[metadata.instance_index % EE_MAX_KERNEL_AMOUNT].xyz);
    const float3 info_kernel = abs(EE_KERNEL_BLOCK.uniform_kernel[metadata.info_index % EE_MAX_KERNEL_AMOUNT].xyz);
    const int debug_visualization = EE_RENDER_INFO.debug_visualization;
    const float mode0 = float(debug_visualization == 0 && metadata.info_index > 1u);
    const float mode1 = float(debug_visualization == 1);
    const float mode2 = float(debug_visualization == 2);
    const float mode3 = float(debug_visualization == 3);
    const float any_debug =
        indirect_lighting_debug_view == 0 ? clamp(mode0 + mode1 + mode2 + mode3, 0.0f, 1.0f) : 0.0f;
    const float3 debug_color = resolved_base_color * mode0 + material_kernel * mode1 + instance_kernel * mode2 +
                               info_kernel * mode3;
    albedo = lerp(resolved_base_color, debug_color, any_debug);
  }

  float3 output_color = indirect_lighting_debug_view == 0 ? albedo : float3(0.0f);
  if (encoded_specular_f90 >= 0.0f) {
    const float3 direct = EE_FUNC_CALCULATE_LIGHTS(
        true, albedo, 1.0f, linear_depth, shading_normal, view_direction, world_position, surface.metallic,
        surface.roughness, surface.specular_f0, encoded_specular_f90);
#ifdef EE_AUTOMATIC_SDFGI
    const float3 ambient = EE_FUNC_CALCULATE_SDFGI_ENVIRONMENTAL_LIGHT(
#else
    const float3 ambient = EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT(
#endif
        albedo, shading_normal, view_direction, world_position, surface.metallic, surface.roughness,
        surface.specular_f0, encoded_specular_f90, resolved_material_ao,
        inAmbientOcclusion.SampleLevel(tex_coord, 0.0f).r);
    if (indirect_lighting_debug_view != 0) {
      outColor[pixel] = float4(ambient, 1.0f);
      return;
    }
    output_color = direct + coated_emissive + ambient;
  }
  if (indirect_lighting_debug_view != 0) {
    outColor[pixel] = float4(0.0f, 0.0f, 0.0f, 1.0f);
    return;
  }

  const float3 sky_color = EE_SKY_COLOR(world_position - camera_position);
  const float fade_ratio = EE_CAMERA_FADE_RATIO(camera_index);
  const float far_distance = EE_CAMERA_FAR(camera_index);
  const float fade_start = far_distance * fade_ratio;
  if (linear_depth > fade_start) {
    const float fade = (linear_depth - fade_start) / (far_distance * (1.0f - fade_ratio));
    output_color = lerp(output_color, sky_color,
                        scene_camera ? clamp(fade, 0.0f, 1.0f) : fade);
  }
  outColor[pixel] = float4(output_color, 1.0f);
}
