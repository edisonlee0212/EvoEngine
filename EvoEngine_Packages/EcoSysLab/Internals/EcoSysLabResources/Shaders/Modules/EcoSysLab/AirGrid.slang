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
