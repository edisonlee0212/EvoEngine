import EcoSysLab.KineticVoronoiMeshing;

[[vk::binding(8, 0)]] public RWStructuredBuffer<SegmentMeshletVertex, Std430DataLayout> segment_meshlet_vertices;
[[vk::binding(9, 0)]] public RWStructuredBuffer<SegmentMeshletTriangle, Std430DataLayout> segment_meshlet_triangles;
