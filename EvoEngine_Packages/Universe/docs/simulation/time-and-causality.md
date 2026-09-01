# Time, Scheduling, and Causality

[Documentation index](../README.md) | Previous: [Universe Generation](../world/generation-and-starts.md) | Next:
[Civilizations, Population, and Economy](civilizations.md)

Status: proposal

## Authoritative Time Step

Simulation advances in deterministic fixed macro-steps. The first prototype should test a one-standard-day quantum.
Every active domain processes the same ordered phases for step `T` before the universe commits step `T + 1`:

1. Deliver information and physical arrivals scheduled for this step.
2. Apply valid local decisions and previously queued orders.
3. Resolve population, production, consumption, maintenance, and environment.
4. Resolve research, construction, institutions, and political pressures.
5. Resolve movement, detection, hazards, and conflict.
6. Generate decisions, missions, observations, and outgoing messages.
7. Record history, checksums, and future scheduled events.

The rendering frame rate never determines simulation results. Fixed stepping supports reproducibility, save comparison,
replay, debugging, and possible future lockstep multiplayer.

## Scalable Stepping

Fixed-step rules do not require equal work from every system every day. Three compatible mechanisms control cost:

- Analytic accumulation integrates stable rates across many steps.
- Scheduled wake-up leaves quiet systems asleep until an arrival, deadline, threshold, or hazard.
- Level of simulation uses detailed models for active systems and validated aggregates for quiet ones.

A sleeping domain always wakes before a causal boundary that could change its result, including arrivals, resource
exhaustion, elections, detected signals, stellar events, and policy deadlines.

## Light Cones

Every observation, message, and physical mission has an origin, destination, creation step, expected arrival step,
propagation speed, payload, and provenance. Communication travels at light speed. Ships, probes, couriers, and weapons
travel below it.

Sensors receive historical emissions after the correct delay. This creates one causal graph for the simulation, AI,
and UI and prevents any subsystem from accidentally using current remote state.

## Player Time Controls

The player can pause and choose increasing simulation rates. Configurable events such as first contact, attack
detection, succession crises, arrivals, and irreversible decisions can automatically pause or slow time.

If the machine cannot sustain a requested rate, the clock advances more slowly instead of dropping steps. At high
speed, local movement animation is presentation and does not replace authoritative resolution.

## Determinism and Persistence

Long campaigns require:

- A stable world seed and independent deterministic random streams per domain or entity.
- Explicit simulation phases and stable ordering for simultaneous events.
- Versioned serialized records with migrations.
- Periodic checksums and deterministic replay tests.
- Snapshot saves plus a bounded event tail.
- Background autosaves and crash-safe replacement of completed saves.

Parallel jobs may calculate independent results, but authoritative commits need deterministic ordering.

## Open Questions

- Is one day the right base quantum, or should longer steps retain sub-step event timestamps?
- Which processes require exact fixed-step resolution rather than analytic integration?
- How far can quiet systems be aggregated without changing political, logistical, or conflict outcomes?
- What replay information is required to diagnose divergence across long campaigns?
