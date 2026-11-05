import EvoEngine.PerFrame;
struct Parameters { uint camera; float time; float duration; float width; float2 viewport; float opacity; float padding; };
struct Output {
  float4 position : SV_Position;
  [[vk::location(0)]] float4 color : COLOR0;
  [[vk::location(1)]] float2 shape : TEXCOORD0;
  [[vk::location(2)]] float4 smoke : TEXCOORD1;
};
[[vk::binding(1, 1)]] StructuredBuffer<uint4> trails;
[[vk::binding(2, 1)]] StructuredBuffer<float4> samples;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;
static const float2 corners[6] = {float2(0,-1),float2(1,-1),float2(1,1),float2(0,-1),float2(1,1),float2(0,1)};
uint Address(uint trail, uint start, uint index) { return trail * 32 + (start + index) % 32; }
[shader("vertex")]
Output main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID) {
  Output output;
  output.position = float4(2,2,2,1);
  output.color = 0;
  output.shape = 0;
  output.smoke = 0;
  uint4 trail = trails[instance_id];
  uint segment = vertex_id / 6;
  if (segment + 1 >= trail.z) return output;
  float4 a = samples[Address(instance_id, trail.y, segment)];
  float4 b = samples[Address(instance_id, trail.y, segment + 1)];
  float4 ca = mul(float4(a.xyz,1), EE_CAMERAS[parameters.camera].projection_view);
  float4 cb = mul(float4(b.xyz,1), EE_CAMERAS[parameters.camera].projection_view);
  if (ca.z <= 0 && cb.z <= 0) return output;
  // Clip crossing segments before perspective division; never expand behind the camera.
  if (ca.z < 0) ca = lerp(ca, cb, saturate(-ca.z / (cb.z - ca.z)));
  if (cb.z < 0) cb = lerp(cb, ca, saturate(-cb.z / (ca.z - cb.z)));
  if (ca.w <= 1e-6 || cb.w <= 1e-6) return output;
  float2 delta = (cb.xy / cb.w - ca.xy / ca.w) * parameters.viewport;
  if (dot(delta,delta) < 1e-8) return output;
  float2 corner = corners[vertex_id % 6];
  float2 tangent = delta;
  uint endpoint = segment + uint(corner.x);
  if (endpoint > 0 && endpoint + 1 < trail.z) {
    float3 before = samples[Address(instance_id, trail.y, endpoint - 1)].xyz;
    float3 after = samples[Address(instance_id, trail.y, endpoint + 1)].xyz;
    float4 cp = mul(float4(before,1), EE_CAMERAS[parameters.camera].projection_view);
    float4 cn = mul(float4(after,1), EE_CAMERAS[parameters.camera].projection_view);
    if (cp.w > 1e-6 && cn.w > 1e-6 && cp.z > 0 && cn.z > 0) {
      float2 joined = (cn.xy / cn.w - cp.xy / cp.w) * parameters.viewport;
      if (dot(joined,joined) > 1e-8) tangent = joined;
    }
  }
  float2 normal = normalize(float2(-tangent.y,tangent.x));
  float fraction = (float(segment) + corner.x) / max(1.0, float(trail.z - 1));
  float sample_time = lerp(a.w,b.w,corner.x);
  float fade = saturate(1.0 - (parameters.time - sample_time) / parameters.duration);
  float speed = length(b.xyz-a.xyz) / max(b.w-a.w,1e-6);
  float3 color = lerp(float3(0.78,0.85,0.9),float3(0.94,0.91,0.83),speed / (speed + 20.0));
  output.position = lerp(ca,cb,corner.x);
  output.position.xy += normal * corner.y * parameters.width * lerp(1.7,0.6,fraction) /
                        max(parameters.viewport,float2(1)) * output.position.w;
  output.color = float4(color * 6.0, parameters.opacity * fade);
  output.shape = float2(corner.y, fraction);
  output.smoke = float4(lerp(a.xyz,b.xyz,corner.x), parameters.time * 0.6 + float(trail.x % 1024) * 0.037);
  return output;
}
