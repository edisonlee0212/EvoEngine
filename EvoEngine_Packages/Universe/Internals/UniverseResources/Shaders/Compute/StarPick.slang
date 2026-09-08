import EvoEngine.PerFrame;

struct StarResult { double4 positionRadius; float4 colorEmission; float4 alphaPadding; };
struct Candidate { double distance; uint index; uint valid; };
struct PickResult { double4 positionRadius; double distance; uint index; uint valid; };
struct Push {
  int cameraIndex; uint count; float2 cursorUv;
  float2 displaySize; float minimumRadius; uint rayValid;
  uint finalPass; uint3 padding;
};
[[vk::push_constant]] ConstantBuffer<Push> constants;
[[vk::binding(0, 1)]] StructuredBuffer<StarResult> stars;
[[vk::binding(1, 1)]] StructuredBuffer<Candidate> candidatesIn;
[[vk::binding(2, 1)]] RWStructuredBuffer<Candidate> candidatesOut;
[[vk::binding(3, 1)]] RWStructuredBuffer<PickResult> result;
[[vk::binding(4, 1)]] Sampler2D sceneDepth;
groupshared Candidate sharedCandidates[256];

Candidate miss() { Candidate c; c.distance = 0.0; c.index = 0xffffffffu; c.valid = 0; return c; }
Candidate nearest(Candidate a, Candidate b) {
  if (a.valid == 0) return b;
  if (b.valid == 0) return a;
  return b.distance < a.distance || (b.distance == a.distance && b.index < a.index) ? b : a;
}

Candidate intersect(uint index) {
  Candidate c = miss();
  if (constants.rayValid == 0 || index >= constants.count) return c;
  const Camera camera = EE_CAMERAS[constants.cameraIndex];
  const double3 relative = stars[index].positionRadius.xyz - double3(camera.inverse_view[3].xyz);
  const double3 center = mul(double4(relative, 0.0), double4x4(camera.view)).xyz;
  const double2 ndc = double2(constants.cursorUv) * 2.0 - 1.0;
  double4 nearPoint = mul(double4(ndc, 0.0, 1.0), double4x4(camera.inverse_projection));
  double4 farPoint = mul(double4(ndc, 1.0, 1.0), double4x4(camera.inverse_projection));
  nearPoint /= nearPoint.w;
  farPoint /= farPoint.w;
  const double3 delta = farPoint.xyz - nearPoint.xyz;
  if (abs(delta.z) < 1e-12) return c;
  const double fraction = (center.z - nearPoint.z) / delta.z;
  if (fraction < 0.0 || fraction > 1.0) return c;
  const double3 point = nearPoint.xyz + delta * fraction;
  const double4 clip = mul(double4(point, 1.0), double4x4(camera.projection));
  if (clip.w <= 0.0) return c;
  const double2 worldPerPixel = 2.0 * abs(clip.w) /
    (abs(double2(camera.projection[0][0], camera.projection[1][1])) * double2(constants.displaySize));
  const double2 radius = max(double2(abs(stars[index].positionRadius.w)),
                             worldPerPixel * double(constants.minimumRadius));
  if (any(radius <= 0.0)) return c;
  const double2 offset = (point.xy - center.xy) / radius;
  if (dot(offset, offset) >= 1.0) return c;
  // Nearest texel, not filtered depth: expanded targets must still respect the cursor's occluder.
  const int2 resolution = int2(camera.resolution_x, camera.resolution_y);
  const int2 pixel = clamp(int2(constants.cursorUv * float2(resolution)), int2(0), resolution - 1);
  const float depth = sceneDepth.Load(int3(pixel, 0)).x;
  if (clip.z / clip.w > double(depth) + 1e-6) return c;
  c.distance = length(point - nearPoint.xyz);
  c.index = index;
  c.valid = 1;
  return c;
}

[shader("compute")]
[numthreads(256, 1, 1)]
void main(uint3 dispatchId : SV_DispatchThreadID, uint3 groupId : SV_GroupID, uint threadId : SV_GroupIndex) {
#if defined(EE_STAR_PICK_REDUCE)
  sharedCandidates[threadId] = dispatchId.x < constants.count ? candidatesIn[dispatchId.x] : miss();
#else
  sharedCandidates[threadId] = intersect(dispatchId.x);
#endif
  GroupMemoryBarrierWithGroupSync();
  for (uint stride = 128; stride != 0; stride >>= 1) {
    if (threadId < stride)
      sharedCandidates[threadId] = nearest(sharedCandidates[threadId], sharedCandidates[threadId + stride]);
    GroupMemoryBarrierWithGroupSync();
  }
  if (threadId == 0) {
    Candidate best = sharedCandidates[0];
    candidatesOut[groupId.x] = best;
    if (constants.finalPass != 0) {
      PickResult r;
      r.positionRadius = best.valid != 0 ? stars[best.index].positionRadius : double4(0.0);
      r.distance = best.distance; r.index = best.index; r.valid = best.valid;
      result[0] = r;
    }
  }
}
