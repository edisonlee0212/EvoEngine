
float EE_INTERLEAVED_GRADIENT_NOISE(vec3 position) {
  vec3 magic = vec3(0.06711056, 0.00583715, 52.9829189);
  return fract(dot(position, magic));
}

vec2 EE_VOGEL_DISK_SAMPLE(int sampleIndex, int sampleCount, float phi) {
  float goldenAngle = 2.4;
  float r = sqrt(float(sampleIndex + 0.5)) / sqrt(float(sampleCount));
  float theta = goldenAngle * sampleIndex + phi;
  return r * vec2(cos(theta), sin(theta));
}

vec2 EE_VOGEL_DISK_SAMPLE(int sampleIndex, int sampleCount, vec3 position) {
  float phi = EE_INTERLEAVED_GRADIENT_NOISE(position);
  return EE_VOGEL_DISK_SAMPLE(sampleIndex, sampleCount, phi);
}
