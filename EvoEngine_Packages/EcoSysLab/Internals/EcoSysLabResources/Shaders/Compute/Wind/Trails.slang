struct Particle { float4 position_age; float4 velocity_lifetime; float4 collision; };
struct Parameters { uint4 emission; float4 limits; uint4 mode; };
[[vk::binding(0, 0)]] StructuredBuffer<Particle> particles;
[[vk::binding(1, 0)]] RWStructuredBuffer<uint4> trails;
[[vk::binding(2, 0)]] RWStructuredBuffer<float4> samples;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;
static const uint sample_count = 32;
uint Address(uint trail, uint start, uint index) { return trail * sample_count + (start + index) % sample_count; }

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  uint slot = id.x;
  if (slot >= 2048) return;
  uint4 trail = trails[slot];
  float now = parameters.limits.x;
  if (parameters.mode.x != 0) {
    if (slot >= parameters.mode.y || trail.z != 0) return;
    uint stride = parameters.emission.w;
    uint first = parameters.emission.y / stride + uint(parameters.emission.y % stride != 0);
    uint ordinal = first + (slot + parameters.mode.y - first % parameters.mode.y) % parameters.mode.y;
    uint sequence = ordinal * stride;
    if (sequence - parameters.emission.y >= parameters.emission.z) return;
    Particle particle = particles[sequence % parameters.mode.z];
    if (particle.velocity_lifetime.w <= 0 || particle.position_age.w >= particle.velocity_lifetime.w) return;
    trails[slot] = uint4(sequence, 0, 1, 1);
    samples[Address(slot, 0, 0)] = float4(particle.position_age.xyz, now);
    return;
  }
  if (trail.z == 0) return;
  if (slot >= parameters.mode.y) trail.w = 0;
  // An occupied trail drains independently of the recycled simulation particle slot.
  if (trail.w != 0) {
    if (parameters.emission.x - trail.x > parameters.mode.z) {
      trail.w = 0;
    } else {
      Particle particle = particles[trail.x % parameters.mode.z];
      float age = particle.position_age.w;
      float lifetime = particle.velocity_lifetime.w;
      bool dead = lifetime <= 0 || age >= lifetime;
      float4 last = samples[Address(slot, trail.y, trail.z - 1)];
      if (lifetime > 0 && (dead || now - last.w >= parameters.limits.w)) {
        float overshoot = max(0.0, age - lifetime);
        float4 sample = float4(particle.position_age.xyz - particle.velocity_lifetime.xyz * overshoot,
                               now - overshoot);
        if (sample.w > last.w) {
          if (trail.z == sample_count) {
            trail.y = (trail.y + 1) % sample_count;
            --trail.z;
          }
          samples[Address(slot, trail.y, trail.z)] = sample;
          ++trail.z;
        }
      }
      if (dead) trail.w = 0;
    }
  }
  float cutoff = now - parameters.limits.y;
  while (trail.z > 1 && samples[Address(slot, trail.y, 1)].w <= cutoff) {
    trail.y = (trail.y + 1) % sample_count;
    --trail.z;
  }
  if (trail.z == 1 && samples[Address(slot, trail.y, 0)].w <= cutoff) {
    trails[slot] = 0;
    return;
  }
  if (trail.z > 1) {
    uint a_index = Address(slot, trail.y, 0);
    float4 a = samples[a_index], b = samples[Address(slot, trail.y, 1)];
    if (a.w < cutoff) samples[a_index] = lerp(a, b, saturate((cutoff - a.w) / max(b.w - a.w, 1e-8)));
    float distance = 0;
    for (uint i = trail.z - 1; i > 0; --i) {
      a_index = Address(slot, trail.y, i - 1);
      a = samples[a_index];
      b = samples[Address(slot, trail.y, i)];
      float segment = length(b.xyz - a.xyz);
      if (distance + segment > parameters.limits.z) {
        samples[a_index] = lerp(b, a, saturate((parameters.limits.z - distance) / max(segment, 1e-8)));
        trail.y = (trail.y + i - 1) % sample_count;
        trail.z -= i - 1;
        break;
      }
      distance += segment;
    }
  }
  trails[slot] = trail;
}
