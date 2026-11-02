import EcoSysLab.TreePhysicsData;

[[vk::binding(4, 0)]] StructuredBuffer<TreePhysicsVertex, Std430DataLayout> vertices;
[[vk::binding(5, 0)]] StructuredBuffer<TreePhysicsRod, Std430DataLayout> rods;

public bool LoadWindRod(uint index, out TreePhysicsRod rod, out TreePhysicsVertex a, out TreePhysicsVertex b) {
  rod = rods[index];
  a = vertices[rod.links.x];
  b = vertices[rod.links.y];
  return true;
}
