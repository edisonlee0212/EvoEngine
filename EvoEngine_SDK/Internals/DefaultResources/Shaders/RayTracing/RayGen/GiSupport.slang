import EvoEngine.DDGI;
import EvoEngine.GiSupport;
import EvoEngine.DDGIProbeRayConstants;
import EvoEngine.DDGIProbeRayPayload;
import EvoEngine.RayTracingBasic;
[[vk::binding(0, 3)]] ConstantBuffer<GiSupportParams> params;
[[vk::binding(1, 3)]] StructuredBuffer<GiSupportProbe> probes;
[[vk::binding(2, 3)]] StructuredBuffer<uint> requests;
[[vk::binding(3, 3)]] RWStructuredBuffer<float4> rays;
[shader("raygeneration")] void main() {
  uint index = DispatchRaysIndex().x, slot = index / GI_SUPPORT_RAYS, ray_index = index % GI_SUPPORT_RAYS;
  if (requests[slot] == 0xffffffffu)
    return;
  GiSupportProbe probe = probes[slot];
  float3 direction = GiSupportDirection(ray_index, params.frame, probe.key.xyz);
  DdgiProbeRayPayload payload = (DdgiProbeRayPayload)0;
  payload.transmission = float3(1);
  payload.seed = (index * 1664525u + params.frame * 1013904223u) & EE_DDGI_RANDOM_SEED_MASK;
  if (payload.seed == EE_DDGI_FIXED_RAY_PAYLOAD_FLAG)
    payload.seed ^= 0xa511e9b3u;
  payload.hit_info.position = probe.position.xyz;
  RayDesc ray = {probe.position.xyz, 0.001, direction, EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.x};
  TraceRay(EE_TLAS, RAY_FLAG_NONE, EE_DDGI_RAY_MASK_GEOMETRY, 0, 0, 0, ray, payload);
  float3 radiance = payload.hit_info.color.a < 0 ? float3(0) : payload.hit_info.color.rgb * payload.transmission;
  rays[index] = float4(all(isfinite(radiance)) ? max(radiance, float3(0)) : float3(0), 1);
}
