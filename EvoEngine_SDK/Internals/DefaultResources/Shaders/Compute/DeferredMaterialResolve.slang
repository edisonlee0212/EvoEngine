import EvoEngine.PerFrame;
import EvoEngine.GltfRasterMaterial;
import EvoEngine.GBuffer;

[[vk::binding(17, 1)]]
Sampler2D inDepth;

[[vk::binding(0, 2)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> inOutUvBaseColorAo;
[[vk::binding(1, 2)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> inOutFrameNormalRoughness;
[[vk::binding(2, 2)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> inOutDdxPbrFlags;
[[vk::binding(3, 2)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> inOutDdyEmissive;
[[vk::binding(4, 2)]]
Texture2D<uint4> inMetadata;

struct DeferredMaterialResolvePushConstant {
  int camera_index;
};

[[vk::push_constant]]
ConstantBuffer<DeferredMaterialResolvePushConstant> constants;

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

  const float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(width, height);
  const float depth = inDepth.SampleLevel(tex_coord, 0.0f).x;
  if (!isfinite(depth) || depth >= 1.0f) {
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

  const float3 world_position = EE_DEPTH_TO_WORLD_POS(constants.camera_index, tex_coord, depth);
  const float3 view_direction = normalize(EE_CAMERA_POSITION(constants.camera_index) - world_position);
  const float3 coated_emissive = EE_GLTF_RASTER_COATED_EMISSION(metadata.material_index, surface, tex_coords,
                                                                geometric_normal, tangent, tangent_handedness, 1.0f,
                                                                view_direction);
  const float encoded_specular_f90 =
      EE_GLTF_MATERIALS[metadata.material_index].unlit != 0 ? -1.0f : surface.specular_f90;

  inOutUvBaseColorAo[pixel] = float4(max(surface.base_color.rgb, float3(0.0f)), max(surface.occlusion, 0.0f));
  inOutFrameNormalRoughness[pixel] = float4(shading_normal, surface.roughness);
  inOutDdxPbrFlags[pixel] = float4(surface.metallic, surface.specular_f0);
  inOutDdyEmissive[pixel] = float4(coated_emissive, encoded_specular_f90);
}
