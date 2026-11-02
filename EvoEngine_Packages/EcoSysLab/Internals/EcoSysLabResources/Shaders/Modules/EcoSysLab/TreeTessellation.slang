// Quantized levels keep angular samples nested until the configured maximum is reached.
public uint TreeTessellationLevel(float required, uint minimum, uint maximum) {
  if (!isfinite(required)) return maximum;
  uint level = minimum;
  while (float(level) < min(required, float(maximum))) level = min(maximum, level * 2u);
  return min(level, maximum);
}

public float TreeClipRadius(float4 center, float4 extent, float2 resolution) {
  if (center.w <= extent.w + 1e-5 || center.z <= extent.z || any(resolution <= 0)) return 1e30;
  float2 radius = (extent.xy + abs(center.xy / center.w) * extent.w) / (center.w - extent.w);
  return max(radius.x * resolution.x, radius.y * resolution.y) * 0.5;
}

public uint TreeScreenRadialCount(float radius, float error, uint maximum) {
  if (error <= 0 || !isfinite(radius) || radius >= 1e29) return maximum;
  float angle = acos(clamp(1.0 - error / max(radius, 1e-6), -1.0, 1.0));
  return TreeTessellationLevel(3.14159265359 / max(angle, 1e-6), 4u, maximum);
}

public uint TreeScreenRingCount(float4 c0, float4 c1, float4 c2, float4 c3,
                               float2 resolution, float error, float edge_pixels, uint maximum) {
  float w = min(min(c0.w, c1.w), min(c2.w, c3.w));
  if (maximum <= 1 || w <= 1e-5 || min(min(c0.z,c1.z),min(c2.z,c3.z)) <= 0 ||
      any(resolution <= 0) || error <= 0 || edge_pixels <= 0) return maximum;
  // Bound the derivatives of the projected rational cubic, with a translated pixel origin.
  float2 origin = c0.xy / c0.w;
  float2 a0 = 0;
  float2 a1 = (c1.xy - origin * c1.w) * resolution * 0.5;
  float2 a2 = (c2.xy - origin * c2.w) * resolution * 0.5;
  float2 a3 = (c3.xy - origin * c3.w) * resolution * 0.5;
  float a = max(max(length(a0),length(a1)),max(length(a2),length(a3)));
  float d1 = 3 * max(max(length(a1-a0),length(a2-a1)),length(a3-a2));
  float d2 = 6 * max(length(a2-2*a1+a0),length(a3-2*a2+a1));
  float w1 = 3 * max(max(abs(c1.w-c0.w),abs(c2.w-c1.w)),abs(c3.w-c2.w));
  float w2 = 6 * max(abs(c2.w-2*c1.w+c0.w),abs(c3.w-2*c2.w+c1.w));
  float speed = d1/w + a*w1/(w*w);
  float curvature = d2/w + (a*w2 + 2*d1*w1)/(w*w) + 2*a*w1*w1/(w*w*w);
  return TreeTessellationLevel(max(speed/edge_pixels, sqrt(curvature/(8*error))), 1u, maximum);
}
