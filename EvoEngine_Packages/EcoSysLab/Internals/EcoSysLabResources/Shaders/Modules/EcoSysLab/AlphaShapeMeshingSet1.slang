import EcoSysLab.AlphaShapeMeshing;

[[vk::binding(0, 3)]] public RWStructuredBuffer<UniformParticle, Std430DataLayout> uniform_particles;
[[vk::binding(1, 3)]] public RWStructuredBuffer<DelaunayTetrahedron, Std430DataLayout> delaunay_tetrahedrons;
