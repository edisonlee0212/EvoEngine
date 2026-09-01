# Player Interface and History

[Documentation index](../README.md) | Previous:
[Knowledge, Contact, Travel, and Conflict](../simulation/knowledge-contact-conflict.md) | Next:
[Technical Direction and Roadmap](../engineering/architecture-and-roadmap.md)

Status: proposal

## Core Loop

The repeating strategic loop is:

1. Receive delayed reports and identify what changed in the known universe.
2. Update beliefs about opportunities, risks, and other societies.
3. Allocate local resources and establish policies, missions, and standing doctrine.
4. Advance time while plans propagate and local agents act.
5. Observe consequences, revise the model, and preserve options against failure.

## Map Scales

The interface moves across four conceptual views:

- Cluster view: radial bands, angular sectors, regions, signals, known powers, and causal distance.
- Network view: travel, communication, logistics, political relationships, and mission timelines.
- System view: stars, worlds, habitats, traffic, defense, and the local economy.
- World view: settlements, biosphere, terrain, infrastructure, and planetary crises.

Detail appears only when meaningful. The player should not click through thousands of similar systems. Filters, alerts,
summaries, standing policy, delegation, and exception-based management are core systems.

## Information State

Fresh local data is crisp. Delayed observations show their source time and age. Predicted state is visually distinct and
includes confidence. Unknown facts remain unknown.

Every remote status view should distinguish:

- Last confirmed state.
- Reports in transit.
- Predicted current state.
- Confidence and known blind spots.
- Earliest time a new observation or order can arrive.

Territory overlays must be observer-relative. The interface cannot reveal a current border, settlement, or fleet merely
because the authoritative simulation knows it exists.

## Timeline and Causality Tools

A first-class timeline should answer:

- When was this fact true?
- When did this observer learn it?
- What is predicted now, and with what confidence?
- When will this order, ship, or signal arrive?
- Which decisions are still reversible?
- What places are inside a selected event's future light cone?

A causal-cone overlay visualizes the earliest possible influence from a selected place and time. This makes light-speed
constraints usable for planning rather than presenting delay as an unexplained restriction.

## History

The simulation records compact, provenance-aware events. Players can follow the life of a colony, technology,
institution, mission, leader, or idea across centuries. Period summaries, maps, lineage graphs, and generated chronicles
turn large-scale state into an understandable history.

Historical map playback should show what changed and what the selected observer knew at that moment. It must not expose
secret historical facts until they are discovered later.

## Required Map Questions

From any system or region, the interface should answer:

- Why is this place strategically useful?
- What role does it perform in the civilization's operational coverage?
- Which radial, tangential, and off-plane routes reach it?
- How long do messages, support, migration, and fleets take?
- What fails if this system becomes unavailable?
- Which facts are old, inferred, disputed, or secret?

## Open Questions

- How many simultaneous territory and knowledge overlays remain readable?
- Should map projections preserve geometry, travel time, or regional relationships at each zoom level?
- How should the game summarize autonomous decisions without overwhelming the player?
- Which events interrupt time automatically, and how customizable is that policy?
