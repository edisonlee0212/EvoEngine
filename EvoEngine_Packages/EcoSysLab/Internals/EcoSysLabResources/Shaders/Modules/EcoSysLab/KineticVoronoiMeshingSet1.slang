import EcoSysLab.KineticVoronoiMeshing;

[[vk::binding(0, 3)]] public RWStructuredBuffer<SegmentMeshletVertex, Std430DataLayout> segment_meshlet_vertices;
[[vk::binding(1, 3)]] public RWStructuredBuffer<SegmentMeshletTriangle, Std430DataLayout> segment_meshlet_triangles;
