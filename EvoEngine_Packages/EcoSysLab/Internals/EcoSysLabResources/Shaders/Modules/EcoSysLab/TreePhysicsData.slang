public struct TreePhysicsVertex {
  public float4 position_inverse_mass;
  public float4 velocity_radius;
  public float4 rest_local;
  public uint4 incident_rods;
};

public struct TreePhysicsRod {
  public uint4 links;
  public float4 length_scale_mass_radius;
  public float4 rotation;
  public float4 rest_rotation;
  public uint4 incident_angles;
};

public struct TreePhysicsAngle {
  public uint4 rods;
  public float4 rest_relative;
  public float4 bend_scale;
};

public struct TreePhysicsBuffers {
  public uint64_t vertices[2];
  public uint64_t rods[2];
  public uint64_t angles;
  public uint64_t vertex_rods;
  public uint64_t rod_angles;
  public uint64_t node_rods;
  public uint64_t predicted;
  public uint64_t previous;
  public uint64_t vertex_coefficients;
  public uint64_t rod_coefficients;
  public uint64_t angle_coefficients;
  public uint64_t reserved;
  public uint4 counts;
};

public struct PhysicsValue<T> {
  public T value;
};

public T PhysicsLoad<T>(uint64_t address, uint index) {
  if (sizeof(T) % 16 == 0)
    return bit_cast<vk::BufferPointer<PhysicsValue<T>, 16>>(address + uint64_t(index) * sizeof(T)).Get().value;
  return bit_cast<vk::BufferPointer<PhysicsValue<T>, 4>>(address + uint64_t(index) * sizeof(T)).Get().value;
}
public void PhysicsStore<T>(uint64_t address, uint index, T value) {
  if (sizeof(T) % 16 == 0)
    bit_cast<vk::BufferPointer<PhysicsValue<T>, 16>>(address + uint64_t(index) * sizeof(T)).Get().value = value;
  else
    bit_cast<vk::BufferPointer<PhysicsValue<T>, 4>>(address + uint64_t(index) * sizeof(T)).Get().value = value;
}

public uint PhysicsAttributeIndex(uint index, uint attribute, uint count, uint stride, bool arrays) {
  return arrays ? attribute * count + index : index * stride + attribute;
}
public TreePhysicsVertex PhysicsVertex(TreePhysicsBuffers buffers, uint copy, uint i) {
  TreePhysicsVertex v;
  uint n = buffers.counts.x;
  bool arrays = buffers.counts.w != 0;
  v.position_inverse_mass = PhysicsLoad<float4>(buffers.vertices[copy], PhysicsAttributeIndex(i,0,n,4,arrays));
  v.velocity_radius = PhysicsLoad<float4>(buffers.vertices[copy], PhysicsAttributeIndex(i,1,n,4,arrays));
  v.rest_local = PhysicsLoad<float4>(buffers.vertices[copy], PhysicsAttributeIndex(i,2,n,4,arrays));
  v.incident_rods = PhysicsLoad<uint4>(buffers.vertices[copy], PhysicsAttributeIndex(i,3,n,4,arrays));
  return v;
}
public TreePhysicsRod PhysicsRod(TreePhysicsBuffers buffers, uint copy, uint i) {
  TreePhysicsRod r;
  uint n = buffers.counts.y;
  bool arrays = buffers.counts.w != 0;
  r.links = PhysicsLoad<uint4>(buffers.rods[copy], PhysicsAttributeIndex(i,0,n,5,arrays));
  r.length_scale_mass_radius = PhysicsLoad<float4>(buffers.rods[copy], PhysicsAttributeIndex(i,1,n,5,arrays));
  r.rotation = PhysicsLoad<float4>(buffers.rods[copy], PhysicsAttributeIndex(i,2,n,5,arrays));
  r.rest_rotation = PhysicsLoad<float4>(buffers.rods[copy], PhysicsAttributeIndex(i,3,n,5,arrays));
  r.incident_angles = PhysicsLoad<uint4>(buffers.rods[copy], PhysicsAttributeIndex(i,4,n,5,arrays));
  return r;
}
