import EcoSysLab.DynamicStrands;

[[vk::binding(0, 0)]] public RWStructuredBuffer<Strand, Std430DataLayout> strands;
[[vk::binding(1, 0)]] public RWStructuredBuffer<Node, Std430DataLayout> nodes;
[[vk::binding(2, 0)]] public RWStructuredBuffer<Segment, Std430DataLayout> segments;
[[vk::binding(3, 0)]] public RWStructuredBuffer<SegmentPair, Std430DataLayout> segment_pairs;
[[vk::binding(4, 0)]] public RWStructuredBuffer<SegmentData, Std430DataLayout> segment_data_list;
[[vk::binding(5, 0)]] public RWStructuredBuffer<HashedGridElement, Std430DataLayout> hashed_grid_elements;
[[vk::binding(6, 0)]] public RWStructuredBuffer<HashedGridCellStart, Std430DataLayout> hashed_grid_cell_starts;
[[vk::binding(7, 0)]] public RWStructuredBuffer<Leaf, Std430DataLayout> foliage;
[[vk::binding(10, 0)]] public RWStructuredBuffer<Particle, Std430DataLayout> segment_particle0s;
[[vk::binding(11, 0)]] public RWStructuredBuffer<Particle, Std430DataLayout> segment_particle1s;
[[vk::binding(12, 0)]] public RWStructuredBuffer<SegmentConnectionHandles, Std430DataLayout>
    segment_connection_handles;
