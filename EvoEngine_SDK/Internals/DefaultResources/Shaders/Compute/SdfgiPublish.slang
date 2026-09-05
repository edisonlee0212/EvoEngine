// Publish only a complete, matching generation; topology failure remains a GPU-authoritative fallback.
import EvoEngine.SdfgiTypes;
[[vk::binding(0, 0)]] ConstantBuffer<SdfgiGatherData> sdfgi;
[[vk::binding(4, 0)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;
[numthreads(1, 1, 1)]
void main() {
  field_status[0].ready = field_status[0].failure_flags == 0 && sdfgi.generation != 0 &&
      field_status[0].generation == sdfgi.generation ? 1 : 0;
}
