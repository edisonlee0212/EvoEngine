# Universe Design Documentation

This directory contains the living design proposal for the strategy game planned for the Universe package. The current
package implements star-cluster visualization and experimental planet terrain; the authoritative strategy simulation
described here has not yet been implemented.

## Recommended Reading Order

1. [Vision and Design Pillars](vision/overview.md)
2. [Strategic Map Design](world/map-design.md)
3. [Universe Generation and Starting Conditions](world/generation-and-starts.md)
4. [Time, Scheduling, and Causality](simulation/time-and-causality.md)
5. [Civilizations, Population, and Economy](simulation/civilizations.md)
6. [Knowledge, Contact, Travel, and Conflict](simulation/knowledge-contact-conflict.md)
7. [Player Interface and History](player-experience/interface-and-history.md)
8. [Technical Direction and Roadmap](engineering/architecture-and-roadmap.md)

## Documentation Hierarchy

```text
docs/
|-- README.md
|-- game-vision.md                    Compatibility entry for the former proposal
|-- vision/
|   `-- overview.md                   Premise, player promise, pillars, and objectives
|-- world/
|   |-- map-design.md                 Spatial structure and expansion incentives
|   `-- generation-and-starts.md      Physical hierarchy, generation, and civilization seeds
|-- simulation/
|   |-- time-and-causality.md         Fixed steps, scheduling, signals, and persistence
|   |-- civilizations.md              Population, economy, governance, and collapse
|   `-- knowledge-contact-conflict.md Exploration, diplomacy, travel, and war
|-- player-experience/
|   `-- interface-and-history.md       Core loop, map views, timelines, and chronicles
`-- engineering/
    `-- architecture-and-roadmap.md    Runtime boundaries, scale budgets, and phases
```

## Document Roles

- Vision documents state the fantasy, values, and product-level constraints.
- World documents define the generated space in which strategy occurs.
- Simulation documents define authoritative rules and interactions.
- Player-experience documents define how those rules become understandable and playable.
- Engineering documents translate design constraints into technical boundaries, measurements, and delivery phases.

Specifications should link upward to the principle they serve and sideways to dependent systems. Implementation plans,
schemas, experiment results, and benchmark reports should become separate documents when they are concrete enough to
change independently.

## Current Foundational Decision

Map design is the first foundational specification. The cluster is understood through two primary strategic axes:

- Radial: travel and expansion between the inner cluster and its outer edge.
- Tangential: travel and expansion clockwise or counterclockwise along an annular band.

These axes organize geography, navigation, regional summaries, and strategic identity. They do not replace continuous
three-dimensional star positions or direct travel with a board-game grid.

## Status Labels

- Proposal: a coherent direction that still needs experiments or product decisions.
- Accepted direction: a constraint to use when designing dependent systems.
- Prototype target: a measurable hypothesis to implement and test.
- Open question: a choice intentionally left unresolved.
