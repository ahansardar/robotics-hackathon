# TurtleBot Pursuit and Evasion: Competition Strategy

This playbook describes the current implementation and the validation standard for entering it in competition. The official source of rules is [RULEBOOK.md](RULEBOOK.md); implementation assumptions are tracked separately in [RULE_COMPLIANCE.md](turtle_pursuit_ws/docs/RULE_COMPLIANCE.md).

## Competitive objective

Round 1 rewards the Catcher with the shortest legal capture time against the organizer's standardized moving target. Round 2 tests both roles in two 180-second innings. A capture requires separation at or below 0.50 m continuously for 1.0 second.

The software therefore optimizes two different outcomes:

- Catcher: reach a feasible intercept quickly, defeat obstacle shielding, then control distance through the full hold interval.
- Runner: avoid predictable motion, preserve open escape routes, exploit legal static obstacles, and survive the complete inning.

## Catcher strategy

The competition Catcher is `predictive`; `aggressive` is its higher-pressure gauntlet variant.

1. Estimate Runner world velocity and yaw rate with a bounded exponential moving average, and track how consistently that yaw rate has held direction (`turn_consistency`).
2. Project curved motion with a constant-turn-rate-and-velocity model, using a forecast horizon scaled down when turn rate is inconsistent (a Runner reacting/juking rather than genuinely arcing).
3. Trust the resulting intercept only if the search actually converged within that horizon; otherwise fall back to chasing the Runner's current position (the switch is debounced, not a per-tick flip).
4. Detect when the Runner is genuinely lingering (not merely passing) near lidar-discovered cover, using a minimum dwell time.
5. Commit to one flank direction around that obstacle instead of aiming through it, capped in duration with a cooldown so a Runner cannot bait an indefinite orbit.
6. Near capture, match Runner speed plus a separation correction so the Catcher remains inside the capture radius for the required second.
7. Pass every tactical target through live lidar corridor selection, boundary recovery, and command limiting.

Success depends on prediction and positioning, not a configured speed advantage: Catcher and Runner share the same default 0.70 m/s ceiling. Measured on the kinematic benchmark, these changes cut `predictive`'s mean capture time from 13.72 s to 12.75 s and closed most (not all) of the gap to `baseline` on strategic/adversarial evasion -- see `docs/CATCHER_ALGORITHM.md` for the full before/after table.

## Runner strategy

The canonical competition Runner is `competitive`, and the Runner gauntlet uses that same algorithm.

1. Score escape headings using distance from the Catcher, open-space and boundary clearance, corner risk, heading continuity, and adversarial break value.
2. Outside a safety-critical breakaway, hold a randomly-weighted choice among the top-scoring headings for a short window instead of always the single best one, so a deterministic policy cannot be fully predicted by an opponent simulating this same scoring function.
3. Use immediate bidirectional motion so a favorable escape behind the robot does not require a slow in-place turn.
4. Trigger high-priority `BREAKAWAY` behavior at close separation -- always the single best escape here, never the mixed-strategy choice.
5. Select cover only from obstacles detected by the Runner's own lidar map.
6. Join a safe orbit around the chosen obstacle and retain it through sensor noise.
7. Switch cover only after the current obstacle is genuinely lost and the replacement is materially better.
8. Cruise while safe and smoothly boost as the Catcher enters the threat range.
9. Replan every control cycle and let live lidar override unsafe tactical commands.

`SHIELD` means using an existing legal arena obstacle as cover. It adds no object or physics advantage.

## Dynamic-world policy

No competition obstacle coordinates are configured. Each robot independently projects current lidar rays into a bounded world-frame occupancy grid. Free rays remove stale cells, compact occupied components produce obstacle-center estimates, and the tactical layer consumes those estimates. The random-world generator changes positions and rotations between runs while preserving start and obstacle spacing.

The system is reactive, but not omniscient. An obstacle outside sensor range cannot influence a command until observed, and no algorithm can guarantee a win in every physically impossible or disconnected layout. Claims must be tied to the tested generator constraints and recorded seeds.

## Speed policy

The supplied rule documents contain no explicit speed limit. Current simulator defaults are:

| Parameter | Both roles |
|---|---:|
| cruise speed | 0.44 m/s |
| boost ceiling | 0.70 m/s |
| linear acceleration | 1.20 m/s² |
| angular-speed ceiling | 1.80 rad/s |

Both roles select speed continuously from tactical demand. Target alignment, lidar clearance, boundary recovery, acceleration, and the hard ceiling remain safety governors. Replace these parameters when the organizer publishes official limits.

## Validation gates

A release candidate is acceptable only after:

1. `colcon build --packages-select turtle_pursuit --symlink-install` succeeds;
2. all package tests pass and `colcon test-result --verbose` reports no failures;
3. deterministic benchmarks run across multiple seeds and plausible Runner behaviors;
4. a full-sensor Gazebo trial passes live stream verification;
5. the hardest matchup runs in a newly generated arena without collision;
6. every failure is replayed using its recorded arena seed;
7. capture decisions agree with the 0.50 m / 1.0 s continuous-hold rule;
8. the result JSON and dashboard metrics agree with observed behavior.

Benchmarks are regression evidence, not proof of universal performance. Record duration, seed, strategies, sensor profile, final state, minimum separation, collisions, and capture or survival time for every published claim.

## Submission checklist

- Build and test from a clean clone on the submission machine.
- Confirm the official technical rulebook against [RULE_COMPLIANCE.md](turtle_pursuit_ws/docs/RULE_COMPLIANCE.md).
- Pin the exact launch command and configuration used in the video.
- Show the dashboard and legal capture-hold progress in video evidence.
- Include randomized layouts and at least one replayed seed, not only a favorable arena.
- Preserve an offline copy of dependencies and the repository for the venue.
- Keep `stable` sensor mode only as an emergency diagnostic fallback; use all required sensors in competition.

## Day-of decision order

1. Run preflight.
2. Build and run tests.
3. Start the requested role and official world/interface.
4. Verify live required topics.
5. Confirm both robots start simultaneously at the required separation.
6. Save result JSON and the arena/configuration identifiers after every inning.

Do not tune against one observed layout by hardcoding it. Change only configuration or general sensing/planning behavior that remains valid when obstacles and opponent motion change.
