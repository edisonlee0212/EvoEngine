// @evoengine-dialect native
struct BrdfInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

static const float PI = 3.14159265359f;

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

float GeometrySchlickGGX(float ndotv, float roughness) {
  float a = roughness;
  float k = (a * a) / 2.0f;
  return ndotv / (ndotv * (1.0f - k) + k);
}

float GeometrySmith(float3 n, float3 v, float3 l, float roughness) {
  float ndotv = max(dot(n, v), 0.0f);
  float ndotl = max(dot(n, l), 0.0f);
  return GeometrySchlickGGX(ndotv, roughness) * GeometrySchlickGGX(ndotl, roughness);
}

float2 IntegrateBRDF(float ndotv, float roughness) {
  float3 v = float3(sqrt(1.0f - ndotv * ndotv), 0.0f, ndotv);
  float a = 0.0f;
  float b = 0.0f;
  float3 n = float3(0.0f, 0.0f, 1.0f);

  static const uint SAMPLE_COUNT = 1024u;
  for (uint i = 0u; i < SAMPLE_COUNT; ++i) {
    float2 xi = Hammersley(i, SAMPLE_COUNT);
    float3 h = ImportanceSampleGGX(xi, n, roughness);
    float3 l = normalize(2.0f * dot(v, h) * h - v);
    float ndotl = max(l.z, 0.0f);
    float ndoth = max(h.z, 0.0f);
    float vdoth = max(dot(v, h), 0.0f);

    if (ndotl > 0.0f) {
      float g = GeometrySmith(n, v, l, roughness);
      float g_vis = (g * vdoth) / (ndoth * ndotv);
      float fc = pow(1.0f - vdoth, 5.0f);
      a += (1.0f - fc) * g_vis;
      b += fc * g_vis;
    }
  }
  return float2(a, b) / float(SAMPLE_COUNT);
}

[shader("fragment")]
float2 main(BrdfInput input) : SV_Target {
  return IntegrateBRDF(input.tex_coord.x, input.tex_coord.y);
}
