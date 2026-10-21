# LSystem

[Back to package index](../README.md)

LSystem is a plant-agnostic graph/grammar runtime package. Its derivation engine separates topology-changing rules from
growth rules that update existing graph data. Supporting modules provide simulation clocks, growth fields, organ
geometry/strand outputs and render publication. Scots Pine types demonstrate a concrete growth model.

- `include/LSystemGraph.hpp`, `ProductionRule.hpp`, `DerivationEngine.hpp`: graph and two-phase rule evaluation.
- `include/LSystemSimulation.hpp`, `SimulationClock.hpp`: simulation orchestration.
- `include/OrganMeshChannel.hpp`, `OrganStrandsChannel.hpp`: generated geometry channels.
- `Editor/`: companion inspectors and authoring integration.

Build with `EVOENGINE_ENABLE_LSystem_PACKAGE=ON` and load it through project package settings or the package manager.
The launcher's `lsystem` profile also loads DigitalAgriculture for compatibility; package availability does not imply
that the starter scene already contains a generated plant. See [runtime packages](../../docs/runtime-packages.md).
