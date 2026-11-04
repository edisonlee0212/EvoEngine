import EcoSysLab.TreePhysicsData;

[[vk::binding(6, 0)]] StructuredBuffer<TreePhysicsBuffers, Std430DataLayout> physics_trees;
[[vk::binding(7, 0)]] StructuredBuffer<uint2> physics_rod_groups;

public bool LoadWindRod(uint index, out TreePhysicsRod rod, out TreePhysicsVertex a, out TreePhysicsVertex b) {
  uint2 group = physics_rod_groups[index / 64];
  TreePhysicsBuffers buffers = physics_trees[group.x];
  uint local = group.y + index % 64;
  if (local >= buffers.counts.y) {
    rod = (TreePhysicsRod)0;
    a = (TreePhysicsVertex)0;
    b = (TreePhysicsVertex)0;
    return false;
  }
  rod = PhysicsRod(buffers, 0, local);
  a = PhysicsVertex(buffers, 0, rod.links.x);
  b = PhysicsVertex(buffers, 0, rod.links.y);
  return true;
}
