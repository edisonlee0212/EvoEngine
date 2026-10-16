import EvoEngine.GltfMaterial;
import EvoEngine.GBuffer;
import EvoEngine.Cameras;
import EvoEngine.BasicConstants;
import EvoEngine.Subsurface;

[[vk::binding(17, 1)]] Sampler2D inDepth;
[[vk::binding(1, 4)]] [vk::image_format("rgba16f")] RWTexture2D<float4> inNormal;
[[vk::binding(4, 4)]] Texture2D<uint4> inMetadata;
[[vk::binding(5, 4)]]
[vk::image_format("rgba32f")] RWTexture2D<float4> inOutColor;
[[vk::binding(6, 4)]] [vk::image_format("rgba16f")] RWTexture2D<float4> inDiffuse;

[shader("compute")][numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width, height;
  inDiffuse.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  if (pixel.x >= width || pixel.y >= height) return;
  const float4 center = inDiffuse[pixel];
  if (center.a <= 0.0f) return;
  const EeGBufferMetadata metadata = EE_GBUFFER_UNPACK_METADATA(inMetadata.Load(int3(pixel, 0)));
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[metadata.material_index];
  const int camera_index = EE_BASIC_CONSTANTS.camera_index;
  const float2 uv = (float2(pixel) + 0.5f) / float2(width, height);
  const float depth = inDepth.SampleLevel(uv, 0).r;
  const float3 position = EE_DEPTH_TO_WORLD_POS(camera_index, uv, depth);
  const float3 normal = inNormal[pixel].xyz;
  const float view_z = abs(mul(float4(position, 1), EE_CAMERAS[camera_index].view).z);
  const float2 pixels_per_unit = abs(float2(EE_CAMERAS[camera_index].projection[0][0],
                                           EE_CAMERAS[camera_index].projection[1][1])) *
                               float2(width, height) / max(2.0f * (abs(EE_CAMERAS[camera_index].projection[3][3]) > 0.5f ? 1.0f : view_z), 1e-6f);
  const float3 distance = max(material.subsurface_distance, float3(1e-6f));
  const uint count = clamp(uint(EE_BASIC_CONSTANTS.light_split_index), 1u, 64u);
  float3 sum = float3(0), weights = float3(0);
  for (uint i = 0; i < count; ++i) {
    const uint channel = i % 3u;
    const float u = (float(i / 3u) + 0.5f) / float((count + 2u - channel) / 3u);
    const float radius = EE_SSS_SAMPLE_RADIUS(u, distance[channel]);
    const float angle = float(i) * 2.39996322973f;
    const int2 sample_pixel = int2(pixel) + int2(round(radius * pixels_per_unit * float2(cos(angle), sin(angle))));
    if (any(sample_pixel < int2(0)) || sample_pixel.x >= int(width) || sample_pixel.y >= int(height)) continue;
    const EeGBufferMetadata neighbor = EE_GBUFFER_UNPACK_METADATA(inMetadata.Load(int3(sample_pixel, 0)));
    if (neighbor.instance_index != metadata.instance_index || neighbor.material_index == EE_GBUFFER_MATERIAL_INDEX_MASK) continue;
    const GltfShadeMaterial neighbor_material = EE_GLTF_MATERIALS[neighbor.material_index];
    if (neighbor_material.subsurface_profile_lo != material.subsurface_profile_lo ||
        neighbor_material.subsurface_profile_hi != material.subsurface_profile_hi ||
        neighbor_material.subsurface_mode != material.subsurface_mode) continue;
    const float4 sample = inDiffuse[sample_pixel];
    if (sample.a <= 0.0f || dot(normal, inNormal[sample_pixel].xyz) <= 0.0f) continue;
    const float2 sample_uv = (float2(sample_pixel) + 0.5f) / float2(width, height);
    const float3 sample_position = EE_DEPTH_TO_WORLD_POS(camera_index, sample_uv, inDepth.SampleLevel(sample_uv, 0).r);
    const float actual_radius = length(sample_position - position);
    // Importance weights include depth separation, preventing blur across distant surfaces.
    const float3 proposal = EE_SSS_AREA_PROFILE(radius, distance);
    const float pdf = max((proposal.r + proposal.g + proposal.b) / 3.0f, 1e-12f);
    const float3 weight = EE_SSS_AREA_PROFILE(max(actual_radius, radius), distance) / pdf * sample.a;
    sum += sample.rgb * weight;
    weights += weight;
  }
  const float3 filtered = float3(weights.r > 0 ? sum.r / weights.r : center.r,
                                 weights.g > 0 ? sum.g / weights.g : center.g,
                                 weights.b > 0 ? sum.b / weights.b : center.b);
  const float3 adjustment = EE_SSS_DIFFUSION_ADJUSTMENT(center.rgb, filtered, material.subsurface_albedo, center.a);
  const float4 color = inOutColor[pixel];
  inOutColor[pixel] = float4(max(color.rgb + adjustment, float3(0)), color.a);
}
