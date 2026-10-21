public struct GpuTreeNode {
  public float4 position_thickness;
  public float4 direction_length;
  public float4 radial_axis_distance;
  public float4 subdivisions;
  public float4 bark;
  public float4 bark_base;
  public uint4 links;
  public float4 foliage;
  public float4 foliage_rotation;
  public uint4 foliage_meta;
};

public struct GpuTreeFoliageParams {
  public float4 size_count;
  public float4 filters;
  public float4 distributions;
  public float4 tropism;
  public uint4 flags;
};

public struct GpuTreeInstanceIndices {
  public uint branch;
  public uint foliage;
};

[[vk::binding(0, 3)]] public StructuredBuffer<GpuTreeNode, Std430DataLayout> tree_nodes;
[[vk::binding(1, 3)]] public StructuredBuffer<GpuTreeInstanceIndices, Std430DataLayout> tree_instance_indices;
[[vk::binding(2, 3)]] public StructuredBuffer<GpuTreeFoliageParams, Std430DataLayout> tree_foliage_params;

[[vk::push_constant]] public cbuffer TreeMeshConstants {
  public uint node_base;
  public uint node_count;
  public uint camera_index;
  public uint pass_index;
  public uint light_index;
  public uint light_face;
  public uint padding0;
  public uint geometry_pass;
};

public uint TreeRadialCount(GpuTreeNode node) {
  uint count = max(4u, uint(ceil(node.position_thickness.w * 3.14159265359 / node.subdivisions.x)));
  return min(256u, (count + 1u) & ~1u);
}

// The same ring evaluator is used for a parent's end and each child's start.
float4 TreePermute(float4 x) { return ((x * 34.0 + 1.0) * x) - floor(((x * 34.0 + 1.0) * x) / 289.0) * 289.0; }
float3 TreeMod289(float3 x) { return x - floor(x / 289.0) * 289.0; }
float4 TreeInvSqrt(float4 x) { return 1.79284291400159 - 0.85373472095314 * x; }

// Direct port of the 3D classic Perlin implementation used by glm::perlin.
public float TreePerlin(float3 p) {
  float3 pi0 = TreeMod289(floor(p));
  float3 pi1 = TreeMod289(pi0 + 1.0);
  float3 pf0 = frac(p), pf1 = pf0 - 1.0;
  float4 ix = float4(pi0.x, pi1.x, pi0.x, pi1.x);
  float4 iy = float4(pi0.y, pi0.y, pi1.y, pi1.y);
  float4 ixy = TreePermute(TreePermute(ix) + iy);
  float4 i0 = TreePermute(ixy + pi0.z), i1 = TreePermute(ixy + pi1.z);
  float4 gx0 = i0 / 7.0, gx1 = i1 / 7.0;
  float4 gy0 = frac(floor(gx0) / 7.0) - 0.5;
  float4 gy1 = frac(floor(gx1) / 7.0) - 0.5;
  gx0 = frac(gx0); gx1 = frac(gx1);
  float4 gz0 = 0.5 - abs(gx0) - abs(gy0), gz1 = 0.5 - abs(gx1) - abs(gy1);
  float4 s0 = step(gz0, 0.0), s1 = step(gz1, 0.0);
  gx0 -= s0 * (step(0.0, gx0) - 0.5); gy0 -= s0 * (step(0.0, gy0) - 0.5);
  gx1 -= s1 * (step(0.0, gx1) - 0.5); gy1 -= s1 * (step(0.0, gy1) - 0.5);
  float3 g000 = float3(gx0.x, gy0.x, gz0.x), g100 = float3(gx0.y, gy0.y, gz0.y);
  float3 g010 = float3(gx0.z, gy0.z, gz0.z), g110 = float3(gx0.w, gy0.w, gz0.w);
  float3 g001 = float3(gx1.x, gy1.x, gz1.x), g101 = float3(gx1.y, gy1.y, gz1.y);
  float3 g011 = float3(gx1.z, gy1.z, gz1.z), g111 = float3(gx1.w, gy1.w, gz1.w);
  float4 n0 = TreeInvSqrt(float4(dot(g000,g000),dot(g010,g010),dot(g100,g100),dot(g110,g110)));
  float4 n1 = TreeInvSqrt(float4(dot(g001,g001),dot(g011,g011),dot(g101,g101),dot(g111,g111)));
  g000 *= n0.x; g010 *= n0.y; g100 *= n0.z; g110 *= n0.w;
  g001 *= n1.x; g011 *= n1.y; g101 *= n1.z; g111 *= n1.w;
  float4 z0 = float4(dot(g000,pf0),dot(g100,float3(pf1.x,pf0.y,pf0.z)),
                     dot(g010,float3(pf0.x,pf1.y,pf0.z)),dot(g110,float3(pf1.x,pf1.y,pf0.z)));
  float4 z1 = float4(dot(g001,float3(pf0.x,pf0.y,pf1.z)),dot(g101,float3(pf1.x,pf0.y,pf1.z)),
                     dot(g011,float3(pf0.x,pf1.y,pf1.z)),dot(g111,pf1));
  float3 f = pf0 * pf0 * pf0 * (pf0 * (pf0 * 6.0 - 15.0) + 10.0);
  float4 z = lerp(z0,z1,f.z);
  float2 yz = lerp(z.xy, z.zw, f.y);
  return 2.2 * lerp(yz.x, yz.y, f.x);
}

public float TreeBark(GpuTreeNode node, float angular, float distance) {
  float a = angular * 6.28318530718;
  float s = sin(a), c = cos(a);
  float bark = node.bark.z * TreePerlin(float3(node.bark.x * s, node.bark.x * c, node.bark.y * distance));
  float base = node.bark_base.z * (1.0 + TreePerlin(float3(node.bark.w * s, node.bark.w * c, 0.0)));
  base *= pow(max(0.0, (node.bark_base.x - distance) / max(node.bark_base.x, 0.0001)), node.bark_base.y);
  return bark + base;
}

public struct TreeRingVertex {
  public float3 position;
  public float3 normal;
  public float3 tangent;
  public float2 uv;
};

public void EvaluateTreeFrame(GpuTreeNode node, float t, out float3 center, out float3 axis, out float3 radial0) {
  axis = normalize(node.direction_length.xyz);
  center = node.position_thickness.xyz + axis * (node.direction_length.w * t);
  if (node.direction_length.w > 1e-6 && (node.subdivisions.z > 0.0 || node.subdivisions.w > 0.0)) {
    float3 p0 = node.position_thickness.xyz;
    float3 p3 = p0 + axis * node.direction_length.w;
    float3 incoming = node.links.x == 0xffffffffu ? axis : tree_nodes[node.links.x].direction_length.xyz;
    float3 p1 = p0 + incoming * (node.direction_length.w * node.subdivisions.z);
    float3 p2 = p3 - axis * (node.direction_length.w * node.subdivisions.w);
    float u = 1.0 - t;
    center = u*u*u*p0 + 3.0*u*u*t*p1 + 3.0*u*t*t*p2 + t*t*t*p3;
    float3 derivative = u*u*(p1-p0) + 2.0*u*t*(p2-p1) + t*t*(p3-p2);
    if (dot(derivative, derivative) > 1e-12) axis = normalize(derivative);
  }
  radial0 = node.radial_axis_distance.xyz - axis * dot(axis, node.radial_axis_distance.xyz);
  radial0 = normalize(dot(radial0, radial0) > 1e-8 ? radial0 : cross(abs(axis.y) < 0.9 ? float3(0,1,0) : float3(1,0,0), axis));
}

public TreeRingVertex EvaluateTreeRing(GpuTreeNode node, float t, uint sample, uint count) {
  float3 center, axis, radial0;
  EvaluateTreeFrame(node, t, center, axis, radial0);
  float angular = float(sample) / float(count);
  float a = angular * 6.28318530718;
  float3 radial = radial0 * cos(a) + cross(axis, radial0) * sin(a);
  float distance = node.radial_axis_distance.w + node.direction_length.w * t;
  TreeRingVertex result;
  result.normal = radial;
  result.tangent = normalize(cross(axis, radial));
  float radius = 0.5 * node.position_thickness.w;
  result.position = center +
                    radial * (radius * (1.0 + TreeBark(node, min(angular, 1.0 - angular), distance)));
  result.uv = float2(angular, distance);
  return result;
}

// Ordered merge of the two angular edge sets; every edge on both rings is used.
public void TreeZipper(uint triangle, uint start_count, uint end_count,
                       out uint3 samples, out uint3 ring_ids) {
  uint lo = 1u, hi = start_count;
  while (lo < hi) {
    uint mid = (lo + hi) / 2u;
    uint rank = mid - 1u + (mid * end_count - 1u) / start_count;
    if (rank < triangle) lo = mid + 1u; else hi = mid;
  }
  uint start_rank = lo - 1u + (lo * end_count - 1u) / start_count;
  if (start_rank == triangle) {
    uint j = (lo * end_count - 1u) / start_count;
    samples = uint3(lo - 1u, j, lo);
    ring_ids = uint3(0u, 1u, 0u);
  } else {
    lo = 1u; hi = end_count;
    while (lo < hi) {
      uint mid = (lo + hi) / 2u;
      uint rank = mid - 1u + (mid * start_count) / end_count;
      if (rank < triangle) lo = mid + 1u; else hi = mid;
    }
    uint k = (lo * start_count) / end_count;
    samples = uint3(k, lo - 1u, lo);
    ring_ids = uint3(0u, 1u, 1u);
  }
}
