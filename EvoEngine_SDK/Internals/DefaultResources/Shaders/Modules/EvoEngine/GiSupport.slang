import EvoEngine.GiSupportOverride;
__exported import EvoEngine.HddagiGather;

public
static const uint GI_SUPPORT_CAPACITY = 8192;
public
static const uint GI_SUPPORT_WAYS = 4;
public
static const uint GI_SUPPORT_RAYS = 16;
public
struct GiSupportProbe {
 public
  int4 key;
 public
  float4 position;
 public
  uint4 state;  // generation, last used frame, accumulated frames, reserved
 public
  float4 sh[9];
};
public
struct GiSupportParams {
 public
  HddagiGatherData field;
 public
  uint2 viewport;
 public
  uint camera_index;
 public
  uint frame;
 public
  uint generation;
 public
  uint3 padding;
 public
  float4 sky_color_energy;
 public
  float4 sky_parameters;
};
public
uint GiSupportBucket(int4 key) {
  uint3 v = uint3(key.xyz) * uint3(73856093u, 19349663u, 83492791u);
  return ((v.x ^ v.y ^ v.z ^ uint(key.w) * 104729u) & (GI_SUPPORT_CAPACITY / GI_SUPPORT_WAYS - 1)) * GI_SUPPORT_WAYS;
}
public
float GiSupportBasis(uint i, float3 n) {
  switch (i) {
    case 0:
      return 0.2820947918;
    case 1:
      return 0.4886025119 * n.y;
    case 2:
      return 0.4886025119 * n.z;
    case 3:
      return 0.4886025119 * n.x;
    case 4:
      return 1.0925484306 * n.x * n.y;
    case 5:
      return 1.0925484306 * n.y * n.z;
    case 6:
      return 0.3153915653 * (3 * n.z * n.z - 1);
    case 7:
      return 1.0925484306 * n.x * n.z;
    default:
      return 0.5462742153 * (n.x * n.x - n.y * n.y);
  }
}
public
float3 GiSupportDirection(uint ray, uint frame, int3 key) {
  uint seed = uint(key.x) * 1973u ^ uint(key.y) * 9277u ^ uint(key.z) * 26699u;
  float u = frac((float(ray) + 0.5) / GI_SUPPORT_RAYS + frac(float(frame) * 0.754877666));
  float phi =
      6.28318530718 * frac(float(ray) * 0.618033989 + float(seed & 65535u) / 65536 + float(frame) * 0.569840291);
  float z = 1 - 2 * u;
  float r = sqrt(max(0, 1 - z * z));
  return float3(r * cos(phi), r * sin(phi), z);
}
public
float3 GiSupportIrradiance(GiSupportProbe probe, float3 normal) {
  float3 value = float3(0);
  for (uint i = 0; i < 9; ++i) {
    float convolution = i == 0 ? 3.14159265359 : i < 4 ? 2.09439510239 : 0.78539816339;
    value += probe.sh[i].rgb * GiSupportBasis(i, normal) * convolution;
  }
  return max(value, float3(0));
}

public
interface IGiSupportRead {
  GiSupportProbe loadProbe(uint slot);
  uint2 loadVoxels(int3 cell);
};
public
interface IGiSupportRequests : IGiSupportRead {
  uint claim(uint slot, uint key);
};
public
float3 GiSupportFieldPosition(GiSupportParams p, float3 world) {
  return world * float3(1, p.field.y_mult, 1) - p.field.anchor_origin;
}
public
float3 GiSupportCell(GiSupportParams p, float3 world, uint cascade) {
  return (GiSupportFieldPosition(p, world) - p.field.cascades.data[cascade].offset) *
         p.field.cascades.data[cascade].to_cell;
}
public
uint GiSupportCascade(GiSupportParams p, float3 world) {
  for (uint c = 0; c < p.field.cascade_count; ++c) {
    float3 cell = GiSupportCell(p, world, c);
    if (all(cell >= 0) && all(cell < float3(p.field.grid)))
      return c;
  }
  return p.field.cascade_count;
}
public
bool GiSupportOccupied<T : IGiSupportRead>(T resources, GiSupportParams p, int3 cell, uint cascade) {
  if (any(cell < 0) || any(cell >= p.field.grid))
    return true;
  int3 wrapped = HddagiWrap(cell + p.field.cascades.data[cascade].region_world_offset * 8, p.field.grid);
  uint2 value = resources.loadVoxels(wrapped / 4 + int3(0, cascade * (p.field.grid.y / 4), 0));
  int3 local = wrapped & 3;
  uint bit = uint(local.x + local.y * 4 + local.z * 16);
  return (value[bit >> 5] & (1u << (bit & 31))) != 0;
}
public
bool GiSupportVisible<T : IGiSupportRead>(T resources, GiSupportParams p, float3 start, float3 end, uint cascade) {
  int3 cell = int3(floor(start)), target = int3(floor(end));
  float3 direction = end - start;
  int3 step = int3(sign(direction));
  float3 delta = 1 / max(abs(direction), float3(1e-8));
  float3 next = select(step > 0, float3(cell + 1) - start, start - float3(cell)) * delta;
  next = select(step == 0, float3(1e30), next);
  for (uint i = 0; i < 48; ++i) {
    if (GiSupportOccupied(resources, p, cell, cascade))
      return false;
    if (all(cell == target))
      return true;
    uint axis = next.x <= next.y && next.x <= next.z ? 0 : next.y <= next.z ? 1 : 2;
    cell[axis] += step[axis];
    next[axis] += delta[axis];
  }
  return false;
}
public
bool GiSupportKey<T : IGiSupportRead>(T resources, GiSupportParams p, float3 world, float3 normal, float bias,
                                      out int4 key, out float3 start) {
  uint cascade = GiSupportCascade(p, world);
  key = int4(0);
  start = float3(0);
  if (cascade == p.field.cascade_count)
    return false;
  start = GiSupportCell(p, world, cascade) + normalize(normal * float3(1, p.field.y_mult, 1)) * bias;
  int3 cell = int3(floor(start));
  key = int4(cell + p.field.cascades.data[cascade].region_world_offset * 8, int(cascade));
  return !GiSupportOccupied(resources, p, cell, cascade) &&
         GiSupportVisible(resources, p, start, float3(cell) + 0.5, cascade);
}
public
uint GiSupportPackKey(GiSupportParams p, int4 key) {
  int3 cell = key.xyz - p.field.cascades.data[key.w].region_world_offset * 8;
  return uint(cell.x + p.field.grid.x * (cell.y + p.field.grid.y * (cell.z + p.field.grid.z * key.w)));
}
public
int4 GiSupportUnpackKey(GiSupportParams p, uint encoded) {
  uint3 grid = uint3(p.field.grid);
  uint cascade = encoded / (grid.x * grid.y * grid.z);
  int3 cell = int3(encoded % grid.x, (encoded / grid.x) % grid.y, (encoded / (grid.x * grid.y)) % grid.z);
  return int4(cell + p.field.cascades.data[cascade].region_world_offset * 8, int(cascade));
}
public
float3 GiSupportWorldPosition(GiSupportParams p, int4 key) {
  float3 cell = float3(key.xyz - p.field.cascades.data[key.w].region_world_offset * 8) + 0.5;
  float3 field = p.field.cascades.data[key.w].offset + cell / p.field.cascades.data[key.w].to_cell;
  return (field + p.field.anchor_origin) / float3(1, p.field.y_mult, 1);
}
public
bool GiSupportRequest<T : IGiSupportRequests>(T resources, GiSupportParams p, int4 key, bool existing_only = false) {
  bool found = false;
  uint bucket = GiSupportBucket(key), slot = bucket, oldest = 0;
  for (uint way = 0; way < GI_SUPPORT_WAYS; ++way) {
    GiSupportProbe probe = resources.loadProbe(bucket + way);
    if (probe.state.x == p.generation && all(probe.key == key)) {
      slot = bucket + way;
      found = true;
      break;
    }
    uint age = probe.state.x == p.generation ? p.frame - probe.state.y : 0xffffffffu;
    if (age > oldest) {
      oldest = age;
      slot = bucket + way;
    }
  }
  if (existing_only && !found)
    return false;
  uint encoded = GiSupportPackKey(p, key);
  for (uint attempt = 0; attempt < GI_SUPPORT_WAYS; ++attempt) {
    uint target = bucket + (slot - bucket + attempt) % GI_SUPPORT_WAYS;
    uint previous = resources.claim(target, encoded);
    if (previous == 0xffffffffu || previous == encoded)
      return true;
  }
  return false;
}
public
bool GiSupportSample<T : IGiSupportRead>(T resources, GiSupportParams p, float3 world, float3 normal,
                                         float3 geometric_normal, float bias, out float3 irradiance) {
  int4 key;
  float3 start;
  irradiance = float3(0);
  float total = 0;
  if (!GiSupportKey(resources, p, world, geometric_normal, bias, key, start))
    return false;
  for (int z = -1; z <= 1; ++z)
    for (int y = -1; y <= 1; ++y)
      for (int x = -1; x <= 1; ++x) {
        int4 neighbor = key + int4(x, y, z, 0);
        uint bucket = GiSupportBucket(neighbor);
        for (uint way = 0; way < GI_SUPPORT_WAYS; ++way) {
          GiSupportProbe probe = resources.loadProbe(bucket + way);
          if (probe.state.x != p.generation || probe.state.z == 0 || any(probe.key != neighbor))
            continue;
          float3 end = GiSupportCell(p, probe.position.xyz, uint(key.w));
          if (!GiSupportVisible(resources, p, start, end, uint(key.w)))
            continue;
          float3 distance = abs(end - start);
          float3 kernel =
              select(distance < 0.5, 0.75 - distance * distance, 0.5 * pow(max(1.5 - distance, float3(0)), float3(2)));
          float weight = kernel.x * kernel.y * kernel.z * min(float(probe.state.z) / 8, 1);
          if (weight <= 0)
            continue;
          irradiance += GiSupportIrradiance(probe, normal) * weight;
          total += weight;
        }
      }
  if (total > 0)
    irradiance /= total;
  return total > 0;
}
public
float GiSupportRefinementWeight(float visibility) {
  return EE_GI_SUPPORT_REFINEMENT_WEIGHT(visibility);
}
