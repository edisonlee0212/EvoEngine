public bool LineCircleIntersect(in float2 P1, in float2 P2, in float2 C, float r) {
  // Early exit: Line segment is a point
  if (distance(P1, P2) < 1e-9f) {
    return length(P1 - C) <= r;
  }

  // Check if at least one endpoint is inside the circle
  if (length(P1 - C) <= r || length(P2 - C) <= r) {
    return true;
  }

  float2 d = P2 - P1;
  float2 f = C - P1;

  // Compute t_closest
  float t_closest = dot(f, d) / dot(d, d);

  // Clamp t to the segment
  float t_segment = clamp(t_closest, 0.0f, 1.0f);

  // Compute closest point
  float2 P_closest = P1 + t_segment * d;

  // Compute distance from closest point to circle center
  // Check if distance is within the radius
  return length(P_closest - C) <= r;
}
