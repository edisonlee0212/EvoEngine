import EcoSysLab.AlphaShapeMeshing;

[[vk::binding(8, 0)]] public RWStructuredBuffer<UniformParticle, Std430DataLayout> uniform_particles;
[[vk::binding(9, 0)]] public RWStructuredBuffer<DelaunayTetrahedron, Std430DataLayout> delaunay_tetrahedrons;
