import EvoEngine.HddagiGather;

struct Input {
  float3 position;
  uint cascade;
  float3 normal;
  float roughness;
  float3 reflection;
  uint dynamic_receiver;
};
[[vk::binding(0, 0)]] ConstantBuffer<HddagiGatherData> data;
[[vk::binding(1, 0)]] Texture2DArray<float3> diffuse_atlas;
[[vk::binding(2, 0)]] Texture2DArray<float3> specular_atlas;
[[vk::binding(3, 0)]] Texture3D<float4> occlusion0;
[[vk::binding(4, 0)]] Texture3D<float4> occlusion1;
[[vk::binding(5, 0)]] SamplerState linear_sampler;
[[vk::binding(6, 0)]] StructuredBuffer<Input> inputs;
[[vk::binding(7, 0)]] RWStructuredBuffer<float4> output;
#ifdef MODE_FULL_GATHER
[[vk::binding(8, 0)]] Texture3D<uint2> voxels;
[[vk::binding(9, 0)]][vk::image_format("r8ui")] Texture3D<uint> regions;
[[vk::binding(10, 0)]] Texture3D<float3> light;
[[vk::binding(11, 0)]][vk::image_format("r8ui")] Texture3D<uint> disocclusion;
[[vk::binding(12, 0)]] Texture3D<uint> neighbors;
#endif

[numthreads(64, 1, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint count, stride;
  inputs.GetDimensions(count, stride);
  if (id.x >= count)
    return;
  Input input = inputs[id.x];
#ifdef MODE_FULL_GATHER
  float4 diffuse, specular;
  HddagiGatherLighting(data, diffuse_atlas, specular_atlas, occlusion0, occlusion1, linear_sampler, voxels, regions,
                       light, disocclusion, neighbors, input.position, input.normal, input.reflection, input.roughness,
                       input.dynamic_receiver != 0, true, uint2(id.x, 0), diffuse, specular);
  output[id.x * 2] = diffuse;
  output[id.x * 2 + 1] = specular;
#else
  float3 diffuse, specular;
  HddagiGatherProbes(data, diffuse_atlas, specular_atlas, occlusion0, occlusion1, linear_sampler, input.cascade,
                     input.position, input.normal, input.reflection, input.roughness, input.dynamic_receiver != 0,
                     diffuse, specular);
  output[id.x * 2] = float4(diffuse, 1);
  output[id.x * 2 + 1] = float4(specular, 1);
#endif
}
