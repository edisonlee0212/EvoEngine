import EcoSysLab.TreePhysicsData;
[[vk::binding(0,0)]] StructuredBuffer<uint64_t> addresses;
[numthreads(1,1,1)]
void main() {
  TreePhysicsVertex a = PhysicsLoad<TreePhysicsVertex>(addresses[0], 1);
  a.position_inverse_mass.y += 3;
  PhysicsStore<TreePhysicsVertex>(addresses[0], 1, a);
  uint value = PhysicsLoad<uint>(addresses[1], 1);
  PhysicsStore<uint>(addresses[1], 2, value + 7);
}
