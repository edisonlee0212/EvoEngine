import EcoSysLab.AlphaShapeMeshing;

[[vk::binding(0, 1)]] public RWStructuredBuffer<UniformParticle, Std430DataLayout> uniform_particles;
[[vk::binding(1, 1)]] public RWStructuredBuffer<DelaunayTetrahedron, Std430DataLayout> delaunay_tetrahedrons;
