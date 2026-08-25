import EcoSysLab.DynamicStrands;

[[vk::binding(0, 1)]] public RWStructuredBuffer<Strand, Std430DataLayout> strands;
[[vk::binding(1, 1)]] public RWStructuredBuffer<Node, Std430DataLayout> nodes;
[[vk::binding(2, 1)]] public RWStructuredBuffer<Segment, Std430DataLayout> segments;
[[vk::binding(3, 1)]] public RWStructuredBuffer<SegmentPair, Std430DataLayout> segment_pairs;
[[vk::binding(4, 1)]] public RWStructuredBuffer<SegmentData, Std430DataLayout> segment_data_list;
[[vk::binding(5, 1)]] public RWStructuredBuffer<HashedGridElement, Std430DataLayout> hashed_grid_elements;
[[vk::binding(6, 1)]] public RWStructuredBuffer<HashedGridCellStart, Std430DataLayout> hashed_grid_cell_starts;
[[vk::binding(7, 1)]] public RWStructuredBuffer<Leaf, Std430DataLayout> foliage;
