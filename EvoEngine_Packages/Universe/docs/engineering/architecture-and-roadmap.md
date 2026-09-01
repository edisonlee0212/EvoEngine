# Technical Direction and Roadmap

[Documentation index](../README.md) | Previous:
[Player Interface and History](../player-experience/interface-and-history.md)

Status: proposal

## Simulation and Presentation Boundary

Strategic state is authoritative. Rendering consumes snapshots and may interpolate or simplify them, but rendered
entities do not own game state.

The current `UniverseLayer` provides useful foundations for large star ECS workloads, procedural cluster display,
instanced star rendering, and editor controls. `PlanetTerrain` and `TerrainChunk` provide an experimental basis for
close planetary presentation and LOD. None of these systems should become the civilization database.

## Proposed Domains

The long-term package needs these responsibilities, though the names do not prescribe C++ types:

- `UniverseDefinition`: immutable seeds and generated physical facts.
- `UniverseState`: clock, changed physical state, and persistent identifiers.
- `Scheduler`: future events, sleeping-domain wake-ups, and deterministic phase order.
- `KnowledgeState`: observer-specific reports, beliefs, uncertainty, and secrecy.
- `CivilizationState`: lineage, institutions, policies, and strategic goals.
- `SystemState`: worlds, settlements, infrastructure, local stocks, and traffic.
- `MissionState`: ships, probes, messages, attacks, and other causal transfers.
- `HistoryStore`: compact events, summaries, provenance, and replay support.
- `PresentationBridge`: snapshots for ECS entities, maps, terrain, and UI.

Concrete APIs should follow small prototypes and measured data-layout needs.

## Initial Scale Budgets

| Area | Prototype target |
| --- | --- |
| Generated stars | 100,000 to 500,000 |
| Persistently changed systems | Proportional to explored and inhabited space |
| Active detailed systems | Hundreds to low thousands, depending on campaign state |
| Quiet-system cost | Near zero between scheduled wake-ups |
| Simulation quantum | One day, pending measurement |
| Reproducibility | Identical checksum for the same seed, orders, version, and step count |
| Save behavior | Snapshot plus bounded event tail; no rendered assets in authoritative state |

Benchmark memory per star, generated system, settlement, mission, and knowledge record before expanding model detail.
Campaign cost should grow mainly with meaningful activity, not total theoretical content.

## Development Phases

### Phase 0: map, causality, and scale experiments

- Maintain the design hierarchy and begin a decision log.
- Generate and analyze radial/tangential topology at 100,000 and 500,000 stars.
- Prototype deterministic steps, scheduled wake-ups, and light-delay messages.
- Measure compact star, route, system, mission, and knowledge records.

Exit gate: a headless process generates a valid 100,000-star map, explains expansion choices, reproduces its checksum,
and advances long empty intervals without scanning every star each step.

### Phase 1: causal cluster prototype

- Deterministic cluster and lazy system generation.
- Clock, scheduler, save, load, pause, and speed controls.
- One homeworld, probes, ships, observations, and light-speed messages.
- Cluster map showing bands, sectors, routes, information age, and arrival timelines.

Exit gate: the player surveys and sends a mission to another star; observation, order, and arrival obey the same causal
model across replay and save/load.

### Phase 2: single-civilization expansion

- Population, resources, production, research, settlement, and logistics.
- Autonomous colony policy and delayed orders.
- Radial spokes, tangential arcs, route infrastructure, divergence, and local failure.
- Exception-based management and historical summaries.

Exit gate: one civilization expands to dozens of systems for centuries without mandatory per-system micromanagement,
and different map shapes produce different strengths and failure modes.

### Phase 3: many civilizations and contact

- AI seeds, knowledge ledgers, signals, first contact, diplomacy, secrecy, and trade.
- Cultural divergence, federations, secession, and successor identity.
- Detection signatures and concealment tradeoffs.

Exit gate: AI civilizations discover and reason about one another using delayed information only, and major decisions
are explainable from their knowledge at the time.

### Phase 4: conflict, collapse, and continuity

- Fleet missions, interception, doctrine, defense, and strategic weapons.
- Regional collapse, archives, recovery, diaspora, and lineage continuation.
- Resilience and legacy reports.

Exit gate: losing a capital can produce extinction, fragmentation, exile, or recovery depending on spatial preparation
and local conditions rather than a scripted game-over.

### Phase 5: deep time and content

- Stellar evolution and rare cluster-scale hazards.
- Mature technologies that preserve causal strategy.
- Ruins, archaeological inference, long historical cycles, and observer mode.
- Performance tuning for thousand-year and longer campaigns.

Exit gate: campaigns remain computationally stable and understandable after their original political structures vanish.

## Validation Principles

Every prototype should answer a design risk. Validation includes:

- Determinism across repeated runs and supported worker counts.
- Headless performance at target cluster sizes.
- Save/load equivalence and schema migration tests.
- Proof that AI and UI cannot access unauthorized current remote state.
- Comparison of detailed and aggregated simulation at boundary cases.
- Player comprehension of map direction, information age, order delay, and local autonomy.
- Histories in which expansion, cooperation, fragmentation, and recovery occur without scripts.

Visual quality matters, but a beautiful cluster does not prove that the strategic map or simulation works.

## Documentation Growth

As prototypes become concrete, add sibling specifications for:

- Map-generation algorithms and benchmark results.
- Deterministic scheduling and event schemas.
- Solar-system and world generation.
- Population, economy, governance, and AI behavior.
- Sensors, knowledge, communication, and diplomacy.
- Travel, logistics, missions, and combat.
- Save format, replay, and campaign history.
- Interface wireframes and usability findings.
