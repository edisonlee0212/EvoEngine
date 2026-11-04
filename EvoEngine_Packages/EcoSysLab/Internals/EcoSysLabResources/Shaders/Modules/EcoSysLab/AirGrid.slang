public struct AirGridAddresses {
  public uint64_t cells;
  public uint64_t values;
};
public struct AirGridCell { public int4 coordinate_next; };
public struct AirGridValue {
  public float4 velocity_temperature; // velocity, kelvin
  public float4 humidity_strength_density; // specific humidity, wind strength, density
};
public struct AirGridCellChunk { public AirGridCell entries[4096]; };
public struct AirGridValueChunk { public AirGridValue entries[4096]; };

public uint AirGridHash(int3 c, uint bucket_count) {
  uint3 h = uint3(c) * uint3(73856093u, 19349663u, 83492791u);
  return (h.x ^ h.y ^ h.z) % bucket_count;
}

public AirGridCell AirGridReadCell(StructuredBuffer<AirGridAddresses> addresses, uint index) {
  vk::BufferPointer<AirGridCellChunk> chunk =
      bit_cast<vk::BufferPointer<AirGridCellChunk>>(addresses[index / 4096].cells);
  return chunk.Get().entries[index % 4096];
}

public AirGridValue AirGridReadValue(StructuredBuffer<AirGridAddresses> addresses, uint index) {
  vk::BufferPointer<AirGridValueChunk> chunk =
      bit_cast<vk::BufferPointer<AirGridValueChunk>>(addresses[index / 4096].values);
  return chunk.Get().entries[index % 4096];
}

public void AirGridWriteValue(StructuredBuffer<AirGridAddresses> addresses, uint index, AirGridValue value) {
  vk::BufferPointer<AirGridValueChunk> chunk =
      bit_cast<vk::BufferPointer<AirGridValueChunk>>(addresses[index / 4096].values);
  chunk.Get().entries[index % 4096] = value;
}

public uint AirGridFind(StructuredBuffer<uint> heads, StructuredBuffer<AirGridAddresses> addresses,
                 int3 coordinate, uint bucket_count) {
  uint index = heads[AirGridHash(coordinate, bucket_count)];
  while (index != 0xffffffffu) {
    AirGridCell cell = AirGridReadCell(addresses, index);
    if (all(cell.coordinate_next.xyz == coordinate)) return index;
    index = asuint(cell.coordinate_next.w);
  }
  return 0xffffffffu;
}

public float4 AirGridSampleWind(StructuredBuffer<uint> wind_heads, StructuredBuffer<AirGridAddresses> wind_addresses,
                                uint bucket_count, float cell_size, float3 midpoint) {
  float3 position = midpoint / cell_size - 0.5;
  int3 base = int3(floor(position));
  float3 fraction = frac(position);
  float4 result = 0;
  float total = 0;
  for (int z = 0; z < 2; ++z)
    for (int y = 0; y < 2; ++y)
      for (int x = 0; x < 2; ++x) {
        uint index = AirGridFind(wind_heads, wind_addresses, base + int3(x, y, z), bucket_count);
        if (index == 0xffffffffu) continue;
        AirGridValue value = AirGridReadValue(wind_addresses, index);
        float weight = (x != 0 ? fraction.x : 1.0 - fraction.x) *
                       (y != 0 ? fraction.y : 1.0 - fraction.y) *
                       (z != 0 ? fraction.z : 1.0 - fraction.z);
        result += weight * float4(value.velocity_temperature.xyz,
                                  value.humidity_strength_density.y * value.humidity_strength_density.z / 1.225);
        total += weight;
      }
  return total > 0.0 ? result / total : 0;
}
