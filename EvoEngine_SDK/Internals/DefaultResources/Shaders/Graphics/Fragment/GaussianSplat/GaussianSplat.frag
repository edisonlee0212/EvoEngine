layout(location = 0) in VS_OUT {
  vec3 Color;
  vec2 LocalCoord;
  flat float Opacity;
  flat float CutoffRadiusSquared;
} fs_in;

layout(location = 0) out vec4 FragColor;

void main() {
  const float radius_squared = dot(fs_in.LocalCoord, fs_in.LocalCoord);
  if (radius_squared > fs_in.CutoffRadiusSquared) {
    discard;
  }
  const float alpha = fs_in.Opacity * exp(-0.5f * radius_squared);
  if (alpha <= 1.0f / 255.0f) {
    discard;
  }
  FragColor = vec4(fs_in.Color * alpha, alpha);
}
