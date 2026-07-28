struct EeCubemapProcessConstants {
  float4x4 projection_view;
  float preset_value;
};

[[vk::push_constant]]
ConstantBuffer<EeCubemapProcessConstants> constants;

struct CubemapFragmentInput {
  [[vk::location(0)]]
  float3 world_pos : TEXCOORD0;
};

[[vk::binding(0, 0)]]
SamplerCube environmentMap;

static const float PI = 3.14159265359f;

float DistributionGGX(float3 n, float3 h, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;
  float ndoth = max(dot(n, h), 0.0f);
  float ndoth2 = ndoth * ndoth;
  float denom = ndoth2 * (a2 - 1.0f) + 1.0f;
  denom = PI * denom * denom;
  return a2 / denom;
}

float RadicalInverse_VdC(uint bits) {
  bits = (bits << 16u) | (bits >> 16u);
  bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
  bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
  bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
  bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
  return float(bits) * 2.3283064365386963e-10f;
}

float2 Hammersley(uint i, uint n) {
  return float2(float(i) / float(n), RadicalInverse_VdC(i));
}

float3 ImportanceSampleGGX(float2 xi, float3 n, float roughness) {
  float a = roughness * roughness;
  float phi = 2.0f * PI * xi.x;
  float cos_theta = sqrt((1.0f - xi.y) / (1.0f + (a * a - 1.0f) * xi.y));
  float sin_theta = sqrt(1.0f - cos_theta * cos_theta);

  float3 h;
  h.x = cos(phi) * sin_theta;
  h.y = sin(phi) * sin_theta;
  h.z = cos_theta;

  float3 up = abs(n.z) < 0.999f ? float3(0.0f, 0.0f, 1.0f) : float3(1.0f, 0.0f, 0.0f);
  float3 tangent = normalize(cross(up, n));
  float3 bitangent = cross(n, tangent);
  return normalize(tangent * h.x + bitangent * h.y + n * h.z);
}

[shader("fragment")]
float4 main(CubemapFragmentInput input) : SV_Target {
  float3 n = normalize(input.world_pos);
  float3 r = n;
  float3 v = r;
  static const uint SAMPLE_COUNT = 1024u;
  float3 prefiltered_color = float3(0.0f);
  float total_weight = 0.0f;

  for (uint i = 0u; i < SAMPLE_COUNT; ++i) {
    float2 xi = Hammersley(i, SAMPLE_COUNT);
    float3 h = ImportanceSampleGGX(xi, n, constants.preset_value);
    float3 l = normalize(2.0f * dot(v, h) * h - v);
    float ndotl = max(dot(n, l), 0.0f);
    if (ndotl > 0.0f) {
      float d = DistributionGGX(n, h, constants.preset_value);
      float ndoth = max(dot(n, h), 0.0f);
      float hdotv = max(dot(h, v), 0.0f);
      float pdf = d * ndoth / (4.0f * hdotv) + 0.0001f;
      uint width;
      uint height;
      uint levels;
      environmentMap.GetDimensions(0, width, height, levels);
      float2 source_size = max(float2(width, height), float2(1.0f));
      float sa_texel = 4.0f * PI / (6.0f * source_size.x * source_size.y);
      float sa_sample = 1.0f / (float(SAMPLE_COUNT) * pdf + 0.0001f);
      float mip_level = constants.preset_value == 0.0f ? 0.0f : 0.5f * log2(sa_sample / sa_texel);
      prefiltered_color += environmentMap.SampleLevel(l, mip_level).rgb * ndotl;
      total_weight += ndotl;
    }
  }

  return float4(prefiltered_color / total_weight, 1.0f);
}
