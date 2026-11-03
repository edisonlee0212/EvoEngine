import EcoSysLab.TreeTessellation;

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
  public float4 tessellation;
  public float4 impostor;
};

public uint TreeRadialCount(GpuTreeNode node) {
  uint count = max(4u, uint(ceil(node.position_thickness.w * 3.14159265359 / node.subdivisions.x)));
  return min(256u, (count + 1u) & ~1u);
}

public struct TreeTaskPayload {
  public uint node_index;
  public uint rings;
  public uint start_radial;
  public uint radial;
  public float first_ring_t;
  public uint full_rings;
};

public void TreeControlPoints(GpuTreeNode node, out float3 p0, out float3 p1, out float3 p2, out float3 p3) {
  float3 axis = normalize(node.direction_length.xyz);
  p0 = node.position_thickness.xyz;
  p3 = p0 + axis * node.direction_length.w;
  if (node.subdivisions.z > 0 || node.subdivisions.w > 0) {
    float3 incoming = node.links.x == 0xffffffffu ? axis : tree_nodes[node.links.x].direction_length.xyz;
    p1 = p0 + incoming * (node.direction_length.w * node.subdivisions.z);
    p2 = p3 - axis * (node.direction_length.w * node.subdivisions.w);
  } else {
    p1 = lerp(p0,p3,1.0/3.0);
    p2 = lerp(p0,p3,2.0/3.0);
  }
}

float TreeBarkBound(GpuTreeNode node) {
  float bound = 4 * abs(node.bark.z);
  if (node.radial_axis_distance.w < node.bark_base.x && node.bark_base.z != 0)
    bound += 5 * abs(node.bark_base.z) * pow(saturate(1-node.radial_axis_distance.w/max(node.bark_base.x,1e-4)),
                                           node.bark_base.y);
  return bound;
}

float4 TreeRadiusExtent(float radius, float4x4 transform) {
  return radius * (abs(transform[0]) + abs(transform[1]) + abs(transform[2]));
}

public uint TreeViewRadialCount(GpuTreeNode node, float4x4 transform, float4 detail) {
  uint maximum = TreeRadialCount(node);
  if (detail.z <= 0 || maximum == 4) return maximum;
  float bark = TreeBarkBound(node);
  float3 p0,p1,p2,p3;
  TreeControlPoints(node,p0,p1,p2,p3);
  float4 extent = TreeRadiusExtent(0.5 * node.position_thickness.w * (1+bark), transform);
  float radius = max(max(TreeClipRadius(mul(float4(p0,1),transform),extent,detail.xy),
                        TreeClipRadius(mul(float4(p1,1),transform),extent,detail.xy)),
                    max(TreeClipRadius(mul(float4(p2,1),transform),extent,detail.xy),
                        TreeClipRadius(mul(float4(p3,1),transform),extent,detail.xy)));
  if (radius*bark > detail.z*0.25) return maximum;
  return TreeScreenRadialCount(radius,detail.z*0.5,maximum);
}

public TreeTaskPayload TreeBuildTask(uint index, float4x4 transform, float4 detail) {
  TreeTaskPayload result = (TreeTaskPayload)0;
  result.node_index = index;
  GpuTreeNode node = tree_nodes[index];
  if (node.links.z == 0xffffffffu || node.direction_length.w <= 1e-6 ||
      dot(node.direction_length.xyz,node.direction_length.xyz) <= 1e-8) return result;
  uint full_radial = TreeRadialCount(node), full_start = full_radial;
  result.radial = TreeViewRadialCount(node,transform,detail);
  result.start_radial = result.radial;
  bool parent_valid = false;
  if (node.links.x != 0xffffffffu) {
    GpuTreeNode parent = tree_nodes[node.links.x];
    parent_valid = parent.links.z != 0xffffffffu && parent.direction_length.w > 1e-6;
    if (parent_valid) {
      full_start = TreeRadialCount(parent);
      result.start_radial = TreeViewRadialCount(parent,transform,detail);
    }
  }
  uint full_rings = min(max(1u,uint(ceil(node.direction_length.w/node.subdivisions.y))),
                       max(1u,(65535u*20u-full_start+full_radial)/(2u*full_radial)));
  result.rings = full_rings;
  result.full_rings = full_rings;
  // Keep the original fork transition, whose end otherwise changes with the ring count.
  result.first_ring_t = parent_valid ? 1.0/float(full_rings) : 0;
  if (detail.z <= 0 || full_rings == 1) return result;
  float3 p0,p1,p2,p3;
  TreeControlPoints(node,p0,p1,p2,p3);
  float4 c0=mul(float4(p0,1),transform), c1=mul(float4(p1,1),transform);
  float4 c2=mul(float4(p2,1),transform), c3=mul(float4(p3,1),transform);
  float bark=TreeBarkBound(node);
  float4 extent=TreeRadiusExtent(0.5*node.position_thickness.w*(1+bark),transform);
  float radius=max(max(TreeClipRadius(c0,extent,detail.xy),TreeClipRadius(c1,extent,detail.xy)),
                   max(TreeClipRadius(c2,extent,detail.xy),TreeClipRadius(c3,extent,detail.xy)));
  if (radius >= 1e29 || radius*bark > detail.z*0.25) return result;
  uint rings=TreeScreenRingCount(c0,c1,c2,c3,detail.xy,detail.z*0.5,detail.w,full_rings);
  result.rings = parent_valid ? min(full_rings,rings+1u) : rings;
  return result;
}

public void TreeSpanParameters(TreeTaskPayload task, uint span, out float t0, out float t1) {
  if (task.first_ring_t > 0 && task.rings != task.full_rings) {
    t0 = span == 0 ? 0 : lerp(task.first_ring_t,1.0,float(span-1u)/float(max(1u,task.rings-1u)));
    t1 = span == 0 ? task.first_ring_t : lerp(task.first_ring_t,1.0,float(span)/float(max(1u,task.rings-1u)));
  } else {
    t0=float(span)/float(task.rings);
    t1=float(span+1u)/float(task.rings);
  }
  if (span+1u == task.rings) t1=1.0;
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
  float bark = 0, base = 0;
  if (node.bark.z != 0)
    bark = node.bark.z * TreePerlin(float3(node.bark.x * s, node.bark.x * c, node.bark.y * distance));
  if (node.bark_base.z != 0) {
    base = node.bark_base.z * (1.0 + TreePerlin(float3(node.bark.w * s, node.bark.w * c, 0.0)));
    base *= pow(max(0.0, (node.bark_base.x - distance) / max(node.bark_base.x, 0.0001)), node.bark_base.y);
  }
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
    float3 p0,p1,p2,p3;
    TreeControlPoints(node,p0,p1,p2,p3);
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


public struct TreeBatchPayload {
  public TreeTaskPayload nodes[32];
  public uint ends[32];
};

public bool TreeBoundsVisible(float3 low,float3 high,float4x4 transform) {
  float3 center=(low+high)*0.5,extent=(high-low)*0.5;
  for(uint i=0;i<6;++i) {
    float4 clip=float4(0,0,0,1);
    if(i<4) clip[i/2]=(i&1)==0 ? 1.0 : -1.0;
    else clip=i==4 ? float4(0,0,1,0) : float4(0,0,-1,1);
    float4 plane=mul(transform,clip);
    if(dot(float4(center,1),plane)+dot(extent,abs(plane.xyz)) < -1e-5) return false;
  }
  return true;
}

public void TreeNodeBounds(uint index,out float3 low,out float3 high) {
  GpuTreeNode node=tree_nodes[index];
  float3 p0,p1,p2,p3;
  TreeControlPoints(node,p0,p1,p2,p3);
  float radius=0.5*node.position_thickness.w*(1+TreeBarkBound(node));
  low=min(min(p0,p1),min(p2,p3))-radius;
  high=max(max(p0,p1),max(p2,p3))+radius;
  if(node.links.x!=0xffffffffu) {
    GpuTreeNode parent=tree_nodes[node.links.x];
    if(parent.links.z!=0xffffffffu && parent.direction_length.w>1e-6) {
      float3 end,axis,radial;
      EvaluateTreeFrame(parent,1,end,axis,radial);
      float parent_radius=0.5*parent.position_thickness.w*(1+TreeBarkBound(parent));
      low=min(low,end-parent_radius);high=max(high,end+parent_radius);
    }
  }
}

public bool TreeNodeVisible(uint index,float4x4 transform) {
  float3 low,high;
  TreeNodeBounds(index,low,high);
  return TreeBoundsVisible(low,high,transform);
}

// Keep angular sample count distinct from zero so the UV seam is never welded.
public uint3 TreeTriangleKeys(TreeTaskPayload task,uint triangle_index) {
  uint first_count=task.start_radial+task.radial;
  uint span=triangle_index<first_count ? 0u : 1u+(triangle_index-first_count)/(2u*task.radial);
  uint triangle=triangle_index<first_count ? triangle_index : (triangle_index-first_count)%(2u*task.radial);
  uint3 samples,rings;
  TreeZipper(triangle,span==0 ? task.start_radial : task.radial,task.radial,samples,rings);
  return (span+rings)*257u+samples;
}

public TreeRingVertex TreeKeyVertex(TreeTaskPayload task,uint key) {
  uint ring=key/257u,sample=key%257u;
  GpuTreeNode node=tree_nodes[task.node_index];
  float t0,t1;
  TreeSpanParameters(task,ring==0 ? 0 : ring-1,t0,t1);
  if(ring==0 && node.links.x!=0xffffffffu) {
    GpuTreeNode parent=tree_nodes[node.links.x];
    if(parent.links.z!=0xffffffffu && parent.direction_length.w>1e-6)
      return EvaluateTreeRing(parent,1,sample,task.start_radial);
  }
  return EvaluateTreeRing(node,ring==0 ? t0 : t1,sample,ring==0 ? task.start_radial : task.radial);
}


public uint TreeMeshletCount(TreeTaskPayload task) {
  uint first=task.start_radial!=task.radial ? (task.start_radial+task.radial+19)/20 : 0;
  uint remaining=task.rings-(first!=0 ? 1u : 0u),strip=2*task.radial;
  return first+(strip<=20 ? (remaining+20/strip-1)/(20/strip) : remaining*((strip+19)/20));
}

// First triangle, triangle count, first span, span count. Small equal-ring strips share complete rings.
public uint4 TreeMeshletRange(TreeTaskPayload task,uint group) {
  uint first_triangles=task.start_radial+task.radial;
  uint first_groups=task.start_radial!=task.radial ? (first_triangles+19)/20 : 0;
  if(group<first_groups) return uint4(group*20,min(20u,first_triangles-group*20),0,1);
  group-=first_groups;
  uint strip=2*task.radial,span=first_groups!=0 ? 1u : 0u;
  if(strip<=20) {
    uint per_group=20/strip;
    span+=group*per_group;
    uint spans=min(per_group,task.rings-span);
    return uint4(span==0 ? 0 : first_triangles+(span-1)*strip,spans*strip,span,spans);
  }
  uint groups=(strip+19)/20;
  span+=group/groups;
  uint offset=(group%groups)*20;
  return uint4((span==0 ? 0 : first_triangles+(span-1)*strip)+offset,min(20u,strip-offset),span,1);
}

public uint4 TreeMeshletBounds(TreeTaskPayload task,uint4 range) {
  // Indexed vector writes lose raster coverage in the mesh stage; keep these updates scalar.
  uint4 bounds=uint4(0xffffffffu,0,0xffffffffu,0);
  uint3 first=TreeTriangleKeys(task,range.x),last=TreeTriangleKeys(task,range.x+range.y-1);
  for(uint i=0;i<3;++i) {
    if(first[i]/257==range.z) { bounds.x=min(bounds.x,first[i]); bounds.y=max(bounds.y,first[i]); }
    else { bounds.z=min(bounds.z,first[i]); bounds.w=max(bounds.w,first[i]); }
    if(last[i]/257==range.z) { bounds.x=min(bounds.x,last[i]); bounds.y=max(bounds.y,last[i]); }
    else { bounds.z=min(bounds.z,last[i]); bounds.w=max(bounds.w,last[i]); }
  }
  return bounds;
}

public uint TreeMeshletVertexCount(TreeTaskPayload task,uint4 range,uint4 bounds) {
  return range.w>1 ? (range.w+1)*(task.radial+1) : bounds.y-bounds.x+bounds.w-bounds.z+2;
}
public uint TreeMeshletVertexKey(TreeTaskPayload task,uint4 range,uint4 bounds,uint vertex) {
  if(range.w>1) return (range.z+vertex/(task.radial+1))*257+vertex%(task.radial+1);
  uint first_count=bounds.y-bounds.x+1;
  return vertex<first_count ? bounds.x+vertex : bounds.z+vertex-first_count;
}
public uint TreeMeshletVertexIndex(TreeTaskPayload task,uint4 range,uint4 bounds,uint key) {
  if(range.w>1) return (key/257-range.z)*(task.radial+1)+key%257;
  return key/257==range.z ? key-bounds.x : bounds.y-bounds.x+1+key-bounds.z;
}
