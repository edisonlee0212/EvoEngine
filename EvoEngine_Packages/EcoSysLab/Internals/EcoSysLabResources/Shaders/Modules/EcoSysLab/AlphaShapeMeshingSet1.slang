import EcoSysLab.AlphaShapeMeshing;

[[vk::binding(8, 1)]] public RWStructuredBuffer<UniformParticle, Std430DataLayout> uniform_particles;
[[vk::binding(9, 1)]] public RWStructuredBuffer<DelaunayTetrahedron, Std430DataLayout> delaunay_tetrahedrons;
