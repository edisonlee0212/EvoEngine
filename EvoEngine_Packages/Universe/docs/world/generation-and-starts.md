# Universe Generation and Starting Conditions

[Documentation index](../README.md) | Previous: [Strategic Map Design](map-design.md) | Next:
[Time, Scheduling, and Causality](../simulation/time-and-causality.md)

Status: proposal

## Physical Hierarchy

The authoritative world uses stable levels:

1. Cluster: global seed, stellar distribution, regions, background history, and broad hazards.
2. Star system: stars, orbital structure, ownership relationships, local traffic, and system events.
3. World: planets, moons, habitats, biospheres, settlements, resources, and infrastructure.
4. Population: aggregated cohorts with location, biology, culture, health, skills, allegiance, and needs.
5. Organization: governments, factions, firms, fleets, research bodies, faiths, and archives.
6. Mission: a bounded physical or informational process moving through space and time.

Every generated star has an addressable solar-system record. The standard generation profile aims for multiple major
planets or planet-scale bodies per system while permitting unusual architectures when the physical model supports them.

Persistent simulation identifiers must not be pointers to rendered objects. A system continues to exist and evolve
without a loaded scene, terrain mesh, or visible star instance.

## Deterministic Cluster Generation

A game seed produces:

- Cluster shape, radial bands, stellar density, age, metallicity, and regional structure.
- Roughly 100,000 to 500,000 stars for the initial target scale.
- Star mass, age, luminosity, lifecycle, and multiple-star relationships.
- Planets, moons, belts, stable orbital zones, and potential habitats.
- Resource distributions derived from formation history and correlated regional fields.
- Habitability, native chemistry, biospheres, rare anomalies, and environmental hazards.
- Optional pre-game civilizations, ruins, probes, migrations, and extinction events.

Generation is reproducible and streamable. Cheap cluster-wide facts are generated eagerly. Detailed system and world
facts are generated lazily from stable sub-seeds, then persisted only after the simulation changes them.

Map topology is not generated after content as an unrelated layer. Density, corridors, voids, resources, hazards, and
starting positions are evaluated together against the requirements in [Strategic Map Design](map-design.md).

## Starting Civilizations

The player chooses a profile or accepts a fully random seed. Candidate homeworlds pass a viability test and a fairness
envelope but need not be symmetric. The setup reveals the broad promise and risk of the profile without exposing the
local map.

Potential profiles include:

- Peripheral exile: sparse neighbors, low early disruption, high autonomy, and constrained routes.
- Crowded cradle: trade and contact opportunities with high observation and conflict risk.
- Harsh refuge: difficult homeworld, strong adaptation pressure, and valuable nearby resources.
- Ancient inheritor: incomplete ruins or knowledge accompanied by visibility and danger.
- Nomadic beginning: a habitat or fleet-based civilization without a secure planetary center.

An exile start should enable quicker undisturbed development through isolation, not grant free progress. Sparse outer
space increases travel cost, limits specialization, delays assistance, and makes each failed colony more significant.

## Background Population

Non-player civilizations can be seeded at different developmental ages. Their separation should consider travel time,
observation time, expansion potential, and shared corridors rather than Euclidean distance alone.

Generation modes should include:

- Empty dawn: civilizations begin within a narrow historical window.
- Sparse peers: a small number of independently developing societies.
- Living cluster: established powers, frontiers, and unsettled regions.
- Inherited ruins: few living peers but extensive evidence of earlier cycles.

## Generation Validation

For each candidate seed, automated analysis should report:

- Viable systems reachable over early, middle, and late expansion ranges.
- Radial and tangential route choices from every civilization seed.
- Distribution of critical resource and habitat categories.
- Corridor redundancy, dead ends, bottlenecks, and isolated pockets.
- Expected first-observation and first-arrival windows between seeds.
- Core, middle-band, and edge opportunity and hazard profiles.
- Whether one starting region has an irreversible advantage before any decisions occur.

Seeds that violate hard playability bounds should be rejected. Unusual but viable asymmetry should remain.
