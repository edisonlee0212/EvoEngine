// Godot HDDAGI voxel packing, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt.
static const float3 HDDAGI_FACING_DIRECTIONS[26] = {
    float3(-1.0, 0.0, 0.0),
    float3(1.0, 0.0, 0.0),
    float3(0.0, -1.0, 0.0),
    float3(0.0, 1.0, 0.0),
    float3(0.0, 0.0, -1.0),
    float3(0.0, 0.0, 1.0),
    float3(-0.5773502691896258, -0.5773502691896258, -0.5773502691896258),
    float3(-0.7071067811865475, -0.7071067811865475, 0.0),
    float3(-0.5773502691896258, -0.5773502691896258, 0.5773502691896258),
    float3(-0.7071067811865475, 0.0, -0.7071067811865475),
    float3(-0.7071067811865475, 0.0, 0.7071067811865475),
    float3(-0.5773502691896258, 0.5773502691896258, -0.5773502691896258),
    float3(-0.7071067811865475, 0.7071067811865475, 0.0),
    float3(-0.5773502691896258, 0.5773502691896258, 0.5773502691896258),
    float3(0.0, -0.7071067811865475, -0.7071067811865475),
    float3(0.0, -0.7071067811865475, 0.7071067811865475),
    float3(0.0, 0.7071067811865475, -0.7071067811865475),
    float3(0.0, 0.7071067811865475, 0.7071067811865475),
    float3(0.5773502691896258, -0.5773502691896258, -0.5773502691896258),
    float3(0.7071067811865475, -0.7071067811865475, 0.0),
    float3(0.5773502691896258, -0.5773502691896258, 0.5773502691896258),
    float3(0.7071067811865475, 0.0, -0.7071067811865475),
    float3(0.7071067811865475, 0.0, 0.7071067811865475),
    float3(0.5773502691896258, 0.5773502691896258, -0.5773502691896258),
    float3(0.7071067811865475, 0.7071067811865475, 0.0),
    float3(0.5773502691896258, 0.5773502691896258, 0.5773502691896258)};

uint HddagiNormalBits(float3 normal) {
  uint bits = 0;
  for (uint i = 0; i < 26; ++i) {
    float d = dot(normal, HDDAGI_FACING_DIRECTIONS[i]);
    if (i < 6 && d > 0.001) bits |= 1u << i;
    if (d > 0.7) bits |= 1u << (i + 6);
  }
  return bits;
}

uint HddagiPackAlbedo(float3 color) {
  uint3 packed = uint3(saturate(color) * float3(31, 63, 31));
  return packed.r | (packed.g << 5) | (packed.b << 11);
}

uint HddagiEncodeRadiance(float3 rgb) {
  rgb = clamp(rgb, float3(0),
    float3(asfloat(0x477F8000u)));
  float maximum = max(max(asfloat(0x37800000u), rgb.r), max(rgb.g, rgb.b));
  float bias = asfloat((asuint(maximum) + 0x07804000u) & 0x7F800000u);
  uint3 mantissa = asuint(rgb + bias);
  uint exponent = (asuint(bias) << 4) + 0x10000000u;
  return exponent | (mantissa.b << 18) | (mantissa.g << 9) | (mantissa.r & 0x1FFu);
}

float3 HddagiDecodeRadiance(uint packed) {
  return float3(packed & 511u, (packed >> 9) & 511u, (packed >> 18) & 511u) * exp2(float(packed >> 27) - 24);
}

uint2 HddagiPackEmission(float3 emitted, float3 normal) {
  float lumas[6];
  float3 total = float3(0);
  for (uint i = 0; i < 6; ++i) {
    float3 light = emitted * max(0.0, dot(normal, HDDAGI_FACING_DIRECTIONS[i]));
    total += light;
    lumas[i] = max(light.r, max(light.g, light.b));
  }
  float luma = max(total.r, max(total.g, total.b));
  uint aniso = 0;
  for (uint i = 0; i < 6; ++i)
    aniso |= min(31u, uint(lumas[i] / max(luma, 1e-20) * 31.0)) << (i * 5);
  return uint2(HddagiEncodeRadiance(total), aniso);
}

struct HddagiProcessVoxel { uint position; uint albedo_normal; uint emission; uint occlusion; };
struct HddagiUpdateBounds { int3 begin; uint cascade; int3 end; uint padding; };
bool HddagiContains(HddagiUpdateBounds bounds, int3 cell, uint cascade) {
  return bounds.cascade == cascade && all(cell >= bounds.begin) && all(cell < bounds.end);
}
