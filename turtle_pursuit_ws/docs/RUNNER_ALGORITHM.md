# Runner Algorithm

## Purpose

The canonical competition Runner uses the `competitive` strategy. Its objective is to remain outside the 0.5 m capture radius for the complete 180-second inning while obeying arena boundaries, avoiding collisions, and continuing to react to the Catcher at 20 Hz.

The algorithm is deterministic for a fixed seed. The same `competitive` implementation is used by `round2`, `runner-gauntlet`, `catcher-gauntlet`, and `hardest`; no easier substitute Runner is used for Catcher validation.

## Inputs and outputs

The Runner consumes:

- Runner and Catcher poses from the configured state adapter.
- RPLidar ranges for local collision avoidance.
- RGB-D freshness as a sensor-health gate and a fallback target observation.
- A persistent obstacle-center map built online from lidar returns.
- A deterministic seed for repeatable fallback maneuver scoring.

It produces a bounded linear and angular velocity command for the Runner's namespaced diff-drive controller and publishes its current mode on `/match/runner_mode`.

## Decision flow

At every control tick, the Runner follows this sequence:

1. Verify required sensor streams and pose freshness.
2. Project lidar hits into world coordinates and update the persistent obstacle map.
3. Select or retain a shielding obstacle from the detected components.
4. Compute a safe target on the obstacle's protective orbit, opposite the Catcher.
5. Drive forward or backward toward that target, whichever avoids an unnecessary turn.
6. Modify the command using live lidar if an obstacle is inside the safety envelope.
7. Apply acceleration and speed limits before publishing.

If required sensors are stale, the Runner stops instead of continuing on an unsafe estimate.

## Persistent obstacle shielding

Lidar hits are transformed from robot coordinates into world coordinates and accumulated in a 0.15 m time-aware grid. Every lidar beam also marks the cells before its return as free, so an empty beam immediately clears a removed or relocated obstacle. Arena-wall returns and returns within the configured opponent-exclusion radius are removed. Connected occupied cells form compact obstacle components; their bounding-box centers become shield candidates. A 15-second upper TTL retains surfaces long enough to combine multiple viewpoints, then removes geometry that has not been observed. No obstacle coordinates are required in `pursuit.yaml`.

For every detected obstacle `o`, the Runner calculates a cover score:

```text
score(o) = distance(o, Catcher) - 1.6 × distance(o, Runner)
```

This favors an obstacle that is reachable by the Runner but not already controlled by the Catcher. Once selected, the obstacle is retained. Small pose noise cannot make the Runner alternate between symmetric obstacles and lose speed.

The Runner changes cover only when both conditions hold:

- The Catcher is at least 0.35 m closer to the current obstacle than the Runner.
- Another obstacle's cover score is better by more than 0.75.

This hysteresis is essential. Earlier versions selected the best obstacle independently on every tick and wasted distance by repeatedly reversing their decision.

## Orbit target

The protective orbit radius is 1.05 m from the selected detected center. The ideal target angle points from the Catcher through the obstacle, placing the obstacle between the two robots.

The target angle is changed in bounded increments:

- Up to 0.65 rad while joining the orbit.
- Up to 0.48 rad after reaching the orbit.

These limits create a tangent-like approach and prevent a straight path through the obstacle. RPLidar remains the final local collision authority.

## Immediate bidirectional escape

The robot may drive backward. If its target is behind its current heading, `drive_to_bidirectional` converts the target into a reverse command instead of stopping to rotate by approximately 180 degrees.

This specifically fixes the starting condition where the Runner faces the Catcher. The original behavior lost most of its opening separation while turning; the current behavior begins escaping immediately.

## Fallback evasion

If shielding is unavailable, the competitive policy evaluates escape headings using:

- Distance from the Catcher.
- Wall and corner clearance.
- Heading smoothness.
- Radial separation under close threat.
- Deterministic perpendicular feints when separation is safe.

Inside the 1.15 m emergency distance, the candidate set is restricted to high-value breakaway directions. The mode becomes `BREAKAWAY`, and heading selection is always the single best-scoring candidate -- no mixed strategy during a safety-critical escape.

## Mixed-strategy heading selection

Outside a breakaway, `strategic`/`adversarial`/`competitive` do not always take the single best-scoring heading. A fully deterministic evasion policy is fully exploitable by any opponent that can simulate this same scoring function, which is a real risk once Round 2 opponents converge on similar architectures. Instead, the Runner holds a randomly-weighted choice among the top-scoring candidates (default: top 3, weighted toward the best) for a short window (default 0.6 s) before re-rolling.

The choice is held for that window rather than re-rolled every control tick on purpose: re-rolling every 50 ms produced motion indistinguishable from reactive juking to the Catcher's own turn-consistency signal (see `CATCHER_ALGORITHM.md`), which paradoxically made the Catcher *more* cautious, not less -- the two mechanisms interact, so this one commits to a choice for long enough to look like a genuine, if less-than-optimal, path segment rather than noise.

## Motion and safety limits

Default Runner limits are:

| Setting | Value |
|---|---:|
| Cruise linear speed | 0.44 m/s |
| Maximum boost speed | 0.70 m/s |
| Maximum angular speed | 1.8 rad/s |
| Linear acceleration | 1.2 m/s² |
| Angular acceleration | 2.8 rad/s² |
| Pose stale timeout | 0.6 s |
| Arena half-width | 5.0 m |
| Boundary margin | 0.55 m |
| Shield radius | 1.05 m |
| Mixed-strategy candidate pool | top 3 |
| Mixed-strategy hold duration | 0.6 s |
| Obstacle-map resolution | 0.15 m |
| Lidar mapping range | 4.5 m |
| Unobserved-cell upper TTL | 15.0 s (free rays clear sooner) |

The Runner cruises while separation is safe and smoothly increases toward full boost as the Catcher enters the configured 3.0 m threat range. The speed is recomputed each control cycle; heading alignment, current lidar clearance, boundary state, and acceleration limits can lower it immediately.

The tactical goal is passed through an adaptive local navigator on every lidar update. It projects the robot's physical corridor along 48 candidate headings, measures collision-free travel distance, and scores clearance, progress toward the escape goal, continuity with the previous choice, and forward-versus-reverse cost. Side persistence prevents left/right oscillation. If commanded motion produces less than the configured displacement during the stall window, the Runner changes side and executes a bounded reverse-and-turn recovery. This layer uses no obstacle coordinates or fixed routes.

The command limiter rejects non-finite values, clamps speed, and rate-limits acceleration. Lidar checks the direction of travel, including the rear sector while reversing. It slows inside the influence range, turns toward the more open side, and backs away from an imminent obstacle. After tactical and lidar processing, a pose-based boundary governor overrides the command and drives inward if braking lag or a partial map estimate crosses the arena safety line.

## Modes

| Mode | Meaning |
|---|---|
| `SHIELD` | Following the persistent protective orbit. |
| `BREAKAWAY` | Executing an emergency escape at close separation. |
| `COMPETITIVE` | Scored competitive fallback when shielding is unavailable. |
| `*/GAP` | Adaptive navigator is following a selected open corridor. |
| `*/RECOVERY` | Stall watchdog is reversing and changing its preferred side. |
| `BOUNDARY_RETURN` | Final arena governor is driving inward. |
| `WAITING` | Pose data is stale; velocity is zero. |
| `SENSOR_FAULT` | A required live sensor is missing; velocity is zero. |

## Validation history

Historical trials include both full-inning survival against an earlier direct-pressure Catcher and capture after the Catcher gained explicit anti-shield planning. In the last full-sensor release gate on random arena seed 909, the unchanged canonical `competitive` Runner entered `SHIELD/RECOVERY`; the aggressive Catcher captured it at 15.099 seconds with 0.352 m minimum separation and zero collisions. This is an intentionally hard regression, not a claim that the Runner must win every matchup, and both gauntlets continue to use this same Runner algorithm.

That Gazebo gate predates the mixed-strategy heading selection described above, which has only been validated on the pure-Python kinematic benchmark so far (see `CATCHER_ALGORITHM.md`'s validated-result table and `SIMULATION_AND_OPERATIONS.md`). Re-run the Gazebo gate before relying on the 15.099 s figure as current.

## Implementation references

- Strategy: [`runner/strategy.py`](../src/turtle_pursuit/turtle_pursuit/runner/strategy.py)
- ROS controller: [`runner/node.py`](../src/turtle_pursuit/turtle_pursuit/runner/node.py)
- Motion and lidar safety: [`control/motion.py`](../src/turtle_pursuit/turtle_pursuit/control/motion.py)
- Dynamic obstacle mapping: [`perception/obstacles.py`](../src/turtle_pursuit/turtle_pursuit/perception/obstacles.py)
- Parameters: [`config/pursuit.yaml`](../src/turtle_pursuit/config/pursuit.yaml)
