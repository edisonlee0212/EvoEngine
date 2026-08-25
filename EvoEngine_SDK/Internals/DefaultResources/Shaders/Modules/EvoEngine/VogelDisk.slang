public float EE_INTERLEAVED_GRADIENT_NOISE(float3 position) {
  const float3 magic = float3(0.06711056, 0.00583715, 52.9829189);
  return frac(dot(position, magic));
}

public float2 EE_VOGEL_DISK_SAMPLE(int sample_index, int sample_count, float phi) {
  const float golden_angle = 2.4;
  const float radius = sqrt(float(sample_index) + 0.5) / sqrt(float(sample_count));
  const float theta = golden_angle * float(sample_index) + phi;
  return radius * float2(cos(theta), sin(theta));
}

public float2 EE_VOGEL_DISK_SAMPLE(int sample_index, int sample_count, float3 position) {
  return EE_VOGEL_DISK_SAMPLE(sample_index, sample_count, EE_INTERLEAVED_GRADIENT_NOISE(position));
}
