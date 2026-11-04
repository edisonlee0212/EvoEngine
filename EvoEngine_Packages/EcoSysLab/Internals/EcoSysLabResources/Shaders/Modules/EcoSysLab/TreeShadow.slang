__exported import EcoSysLab.GpuTreeMesh;

public float3 TreeShadowLocalVector(float3 value, float3 x, float3 y, float3 z) {
  float determinant = dot(x, cross(y,z));
  return float3(dot(value,cross(y,z)), dot(value,cross(z,x)), dot(value,cross(x,y))) / determinant;
}

public float TreeShadowSphere(float3 origin, float3 direction, float3 center, float radius) {
  float3 relative = origin - center;
  float b = dot(relative, direction);
  float discriminant = b * b - dot(relative, relative) + radius * radius;
  if (discriminant < 0) return 1e30;
  float root = sqrt(discriminant);
  float near = -b - root, far = -b + root;
  return near >= 0 ? near : (far >= 0 ? far : 1e30);
}

// Union of a finite tapered cylinder and its spherical end caps. Direction is normalized.
public float TreeShadowIntersect(float3 origin, float3 direction, float4 start, float4 end) {
  if (dot(origin - start.xyz, origin - start.xyz) <= start.w * start.w ||
      dot(origin - end.xyz, origin - end.xyz) <= end.w * end.w) return 0;
  float hit = min(TreeShadowSphere(origin, direction, start.xyz, start.w),
                  TreeShadowSphere(origin, direction, end.xyz, end.w));
  float3 segment = end.xyz - start.xyz;
  float segment_length = length(segment);
  if (segment_length <= 1e-6) return hit;
  float3 axis = segment / segment_length;
  float3 relative = origin - start.xyz;
  float y = dot(relative, axis), dy = dot(direction, axis);
  float3 radial = relative - y * axis, dr = direction - dy * axis;
  float slope = (end.w - start.w) / segment_length;
  float radius = start.w + slope * y;
  float a = dot(dr, dr) - slope * slope * dy * dy;
  float b = dot(radial, dr) - radius * slope * dy;
  float c = dot(radial, radial) - radius * radius;
  if (y >= 0 && y <= segment_length && c <= 0) return 0;
  float discriminant = b * b - a * c;
  if (abs(a) <= 1e-8) {
    if (abs(b) > 1e-8) {
      float t = -c / (2 * b), height = y + t * dy;
      if (t >= 0 && height >= 0 && height <= segment_length) hit = min(hit, t);
    }
  } else if (discriminant >= 0) {
    float root = sqrt(discriminant);
    for (uint i = 0; i < 2; ++i) {
      float t = (-b + (i == 0 ? -root : root)) / a;
      float height = y + t * dy;
      if (t >= 0 && height >= 0 && height <= segment_length) hit = min(hit, t);
    }
  }
  return hit;
}

public uint TreeShadowPoints(GpuTreeNode node, out float4 points[4]) {
  float3 center, axis, radial;
  EvaluateTreeFrame(node, 0, center, axis, radial);
  points[0] = float4(center, 0.5 * node.position_thickness.w);
  bool parent_valid = false;
  if (node.links.x != 0xffffffffu) {
    GpuTreeNode parent = tree_nodes[node.links.x];
    parent_valid = parent.links.z != 0xffffffffu && parent.direction_length.w > 1e-6;
    if (parent_valid) {
      EvaluateTreeFrame(parent, 1, center, axis, radial);
      points[0] = float4(center, 0.5 * parent.position_thickness.w);
    }
  }
  bool curved = node.subdivisions.z > 0 || node.subdivisions.w > 0;
  float t = parent_valid ? 1.0 / max(1.0, ceil(node.direction_length.w / node.subdivisions.y)) : (curved ? 0.5 : 1.0);
  EvaluateTreeFrame(node, t, center, axis, radial);
  points[1] = float4(center, 0.5 * node.position_thickness.w);
  uint count = 2;
  if (t < 1) {
    if (parent_valid && curved) {
      EvaluateTreeFrame(node, (t + 1) * 0.5, center, axis, radial);
      points[count++] = float4(center, 0.5 * node.position_thickness.w);
    }
    EvaluateTreeFrame(node, 1, center, axis, radial);
    points[count++] = float4(center, 0.5 * node.position_thickness.w);
  }
  for (uint i = count; i < 4; ++i) points[i] = points[count - 1];
  return count;
}

public struct TreeShadowOutput {
  public float4 position : SV_Position;
  [[vk::location(0)]] public float3 origin : TEXCOORD0;
  [[vk::location(1)]] public nointerpolation float3 direction : TEXCOORD1;
  [[vk::location(2)]] public nointerpolation float4 p0 : TEXCOORD2;
  [[vk::location(3)]] public nointerpolation float4 p1 : TEXCOORD3;
  [[vk::location(4)]] public nointerpolation float4 p2 : TEXCOORD4;
  [[vk::location(5)]] public nointerpolation float4 p3 : TEXCOORD5;
  [[vk::location(6)]] public nointerpolation uint2 instance_count : TEXCOORD6;
};

public struct TreeShadowPayload { public uint node_indices[32]; };

// The capsule chain lies inside the convex hull of its endpoint spheres.
public bool TreeShadowVisible(float4 points[4], uint count, float4x4 transform) {
  for (uint plane = 0; plane < 6; ++plane) {
    float4 clip_plane = float4(0,0,0,1);
    if (plane < 4) clip_plane[plane / 2] = (plane & 1) == 0 ? 1.0 : -1.0;
    else clip_plane = plane == 4 ? float4(0,0,1,0) : float4(0,0,-1,1);
    float4 local_plane = mul(transform,clip_plane);
    float radius_scale = length(local_plane.xyz);
    bool outside = true;
    for (uint i = 0; i < count; ++i)
      outside = outside && dot(float4(points[i].xyz,1),local_plane) + points[i].w * radius_scale < -1e-5;
    if (outside) return false;
  }
  return true;
}
