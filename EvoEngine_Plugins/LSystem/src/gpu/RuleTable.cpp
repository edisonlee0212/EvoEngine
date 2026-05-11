#include "gpu/RuleTable.hpp"

#if defined(LSYSTEM_GPU_PIPELINE)

#include "MaizeTasselModules.hpp"  // for TasselSymbol IDs

namespace l_system_plugin::gpu {

RuleTable BuildTasselRuleTable() {
  RuleTable table;
  table.rules.reserve(static_cast<size_t>(TasselRuleKind::Count));

  auto add = [&](TasselRuleKind kind,
                 int32_t predecessor,
                 uint32_t priority,
                 uint32_t max_succ,
                 uint32_t salt) {
    RuleDescriptor d;
    d.rule_kind          = static_cast<uint32_t>(kind);
    d.predecessor_symbol = predecessor;
    d.priority           = priority;
    d.max_successors     = max_succ;
    d.rng_salt           = salt;
    d.param_block_offset = 0;  // Phase 3b
    d.param_block_size   = 0;
    table.rules.push_back(d);
  };

  // Mirrors MaizeTasselRules.hpp rule registration order EXACTLY.
  // Salt values copied verbatim from the MakeNodeRng() literals.
  add(TasselRuleKind::Order0AxisExtension,    TasselSymbol::Apex,      0, 4, 0xC8013EA4u);
  add(TasselRuleKind::Order0AxisExhaustion,   TasselSymbol::Apex,      1, 0, 0u);          // no RNG draws
  add(TasselRuleKind::SpikeApexExtension,     TasselSymbol::SpikeApex, 0, 4, 0xAD90777Du);
  add(TasselRuleKind::SpikeApexTerminal,      TasselSymbol::SpikeApex, 1, 1, 0x417B8EF9u);
  add(TasselRuleKind::LateralApexExtension,   TasselSymbol::Apex,      0, 4, 0xD4E12C77u);
  add(TasselRuleKind::LateralApexTerminal,    TasselSymbol::Apex,      1, 1, 0x9E3779B1u);
  add(TasselRuleKind::LateralBudExpansion,    TasselSymbol::Lateral,   0, 2, 0xB5297A4Du);

  return table;
}

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
