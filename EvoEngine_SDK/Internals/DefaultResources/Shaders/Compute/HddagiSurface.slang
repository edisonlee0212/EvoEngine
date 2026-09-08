import EvoEngine.GltfRasterMaterial;
import EvoEngine.GBuffer;
import EvoEngine.LightingFixedSet3;
import EvoEngine.HddagiCamera;

[[vk::binding(17, 1)]] Sampler2D inDepth;
[[vk::binding(0, 4)]][vk::image_format("rgba16f")] RWTexture2D<float4> rawUv;
[[vk::binding(1, 4)]][vk::image_format("rgba16f")] RWTexture2D<float4> rawFrame;
[[vk::binding(2, 4)]][vk::image_format("rgba16f")] RWTexture2D<float4> rawDdx;
[[vk::binding(3, 4)]][vk::image_format("rgba16f")] RWTexture2D<float4> rawDdy;
[[vk::binding(4, 4)]] Texture2D<uint4> inMetadata;

[numthreads(16, 16, 1)] void main(uint3 id : SV_DispatchThreadID) {
  uint2 pixel = id.xy;
  if (any(pixel >= EE_HDDAGI_CAMERA.viewport))
    return;
  EE_HDDAGI_SURFACE_OUT[pixel] = float4(0);
  float depth = inDepth.SampleLevel((float2(pixel) + 0.5) / float2(EE_HDDAGI_CAMERA.viewport), 0).x;
  if (!isfinite(depth) || depth >= 1)
    return;
  EeGBufferMetadata metadata = EE_GBUFFER_UNPACK_METADATA(inMetadata.Load(int3(pixel, 0)));
  if (EE_GLTF_MATERIALS[metadata.material_index].unlit != 0)
    return;
  float4 uv = rawUv[pixel], frame = rawFrame[pixel], dx = rawDdx[pixel], dy = rawDdy[pixel];
  GltfTexCoords coordinates = EE_GLTF_MAKE_TEX_COORDS_WITH_GRADIENTS(uv.xy, uv.zw, dx.xy, dx.zw, dy.xy, dy.zw);
  GltfRasterMaterial surface =
      EE_EVALUATE_GLTF_RASTER_SURFACE(metadata.material_index, coordinates, metadata.vertex_color);
  float3 normal =
      EE_EVALUATE_GLTF_RASTER_NORMAL(metadata.material_index, coordinates, EE_GBUFFER_OCT_DECODE(frame.xy),
                                     EE_GBUFFER_OCT_DECODE(frame.zw), metadata.negative_tangent_handedness ? -1 : 1);
  EE_HDDAGI_SURFACE_OUT[pixel] =
      float4(normal, surface.roughness + (EE_HDDAGI_RECEIVER_CLASS[metadata.instance_index] != 0 ? 2 : 0));
}
