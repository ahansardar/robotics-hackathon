# Catcher Algorithm

## Purpose

The competition Catcher uses the `predictive` strategy. Its objective is to enter the 0.5 m capture radius around the Runner and remain there continuously for one second. It must defeat the exact canonical `competitive` Runner used in the Runner gauntlet, including its obstacle-shielding behavior.

The Catcher combines curved-motion prediction, close-range capture control, explicit anti-shield flanking, live lidar avoidance, RGB-D fallback tracking, and bounded differential-drive commands.

## Inputs and outputs

The Catcher consumes:

- Catcher and Runner poses from the configured state adapter.
- Estimated Runner world velocity and turn rate.
- RPLidar ranges for obstacle safety.
- RGB and depth observations for sensor health and fallback tracking.
- A persistent obstacle-center map built independently from the Catcher's lidar.

It publishes bounded velocity commands to the Catcher's namespaced diff-drive controller and its current mode on `/match/catcher_mode`.

## Decision flow

At every 20 Hz control tick, the Catcher performs:

1. Required-sensor and pose-freshness checks.
2. World-project lidar hits and update the persistent obstacle map.
3. Exponential smoothing of Runner velocity and turn rate.
4. Close-range capture control if separation is at or below 0.55 m.
5. Shield detection and persistent flank planning when the Runner is near a detected obstacle.
6. Curvature-aware interception in open space.
7. Lidar safety adjustment, while distinguishing the Runner surface from a wall.
8. Speed and acceleration limiting before command publication.

## Runner motion estimation

The velocity estimator derives world-frame `vx`, `vy`, and yaw rate from consecutive Runner poses, then applies an exponential moving average with default weight 0.35.

Including yaw rate is important. A constant-velocity predictor projects a circling Runner along its tangent and repeatedly aims outside the true orbit. The current predictor uses a constant-turn-rate-and-velocity model.

For near-zero turn rate, projection is linear:

```text
x(t) = x₀ + vx × t
y(t) = y₀ + vy × t
```

For a measurable turn rate, the velocity vector is integrated along an arc.

The estimator also tracks `turn_consistency`: the fraction of recent consecutive raw turn-rate samples that agree in sign. A genuinely arcing Runner keeps this near 1.0; a Runner reacting tick-to-tick (juking, evaluating a new escape heading every cycle) drives it toward 0.0. This is a lightweight read of *how the opponent is currently behaving*, used below to decide how far to trust the forecast.

## Open-space interception

The predictor samples future Runner positions from the present to the configured horizon. It selects the earliest position satisfying:

```text
distance(Catcher, predicted Runner) / Catcher speed ≤ prediction time + tolerance
```

The default predictive horizon is 4.0 seconds with a 0.2-second step, but the *effective* horizon used each tick is scaled by `turn_consistency` (down to a configurable floor, default 0.25 of nominal). A constant-turn-rate forecast is only trustworthy while the turn rate is actually persisting in one direction; extrapolating a flip-flopping turn rate several seconds ahead sends the aim point further from the truth than a short-horizon guess would. The target is replanned every control tick, so it adapts continuously rather than committing to a stale intercept.

**Self-arbitration.** The interception search itself reports whether it actually found a time-consistent convergence within the horizon (`feasible`), or exhausted the horizon without one and is just returning the endpoint as an extrapolation guess. The Catcher only steers at the forecast when it is feasible; an infeasible result falls back to `CHASE` (aiming directly at the Runner's current position) instead of trusting an arbitrary guess. The trust switch itself is debounced (a smoothed threshold, not a single-tick flip) so an oscillating Runner cannot make the aim point chatter between the two every cycle. `CHASE` is therefore not baseline-only: `predictive` and `aggressive` fall into it whenever the forecast is not currently trustworthy.

## Anti-shield flanking

Direct interception cannot defeat persistent obstacle shielding. It creates an equilibrium: the Catcher approaches one side while the Runner remains opposite the obstacle. The Catcher therefore detects when the Runner is within 1.65 m of a lidar-mapped obstacle and switches to `FLANK` -- but only after the Runner has lingered near that obstacle for a minimum dwell time (default 0.35 s), and it caps how long a single commitment may run (default 3.0 s) before giving up and cooling down (default 1.5 s). Triggering on raw single-tick proximity previously made the Catcher detour around any obstacle the Runner merely passed near while evading, which measurably cost several seconds against strategic/adversarial Runners; the dwell gate and cap fix that without reintroducing the original circular-orbit failure mode.

The Catcher builds its own map; it does not receive the Runner's map or depend on cooperation from an opponent. Lidar hits are accumulated in a 0.15 m time-aware world grid, arena walls and the tracked Runner are filtered out, and compact connected components provide obstacle centers. Each beam clears free grid cells before its return, so moved or removed obstacles are erased as soon as lidar sees through their old location. A bounded 15-second history combines surfaces seen from different viewpoints for more accurate centers.

The flank planner:

1. Identifies the obstacle nearest the Runner.
2. Requires the Runner to have stayed within trigger range of that same obstacle for the dwell period before committing.
3. Measures the Catcher and Runner angles around that obstacle.
4. Selects a clockwise or counter-clockwise direction using observed Runner angular motion, falling back to the shortest angular route.
5. Retains that direction while the Runner uses the same obstacle, and while the commitment stays under the max-duration cap.
6. Places the next target 0.72 rad ahead on a 1.12 m orbit.

Direction persistence prevents oscillation. Instead of pointing through the obstacle, the Catcher advances around it and uses its higher maximum speed to reduce angular separation. If the Runner abandons that obstacle, normal interception resumes or a new flank is initialized; if a single commitment runs past the cap, the Catcher gives up on it and cools down rather than orbiting indefinitely.

## Close-range capture controller

At 0.55 m separation or less, the mode becomes `CAPTURE`. The Catcher computes a short-horizon intercept and chooses a speed that is at least:

- The configured capture speed.
- The estimated Runner speed plus a correction based on excess separation.

This prevents the Catcher from entering the capture radius and then falling behind before completing the required one-second hold.

## Target-aware lidar safety

Generic obstacle avoidance can misclassify the Runner itself as an obstacle and stop the Catcher just outside capture range. The Catcher supplies the expected Runner bearing and surface range to the lidar filter. Returns consistent with the Runner are excluded from wall avoidance, while other returns still trigger slowing, turning, or retreat.

This exception does not disable lidar. It applies only to a narrow angular and range window around the tracked Runner.

The Catcher's adaptive local navigator evaluates 48 body-width travel corridors on every scan. It balances progress toward the current intercept against available clearance and continuity with its previous choice, so it commits to an open side instead of alternating reactively. Its role-specific weights favor goal progress more strongly than the Runner while retaining a 0.29 m collision corridor. A motion-history watchdog detects when a nonzero command fails to move the robot, switches the preferred side, and performs a bounded reverse-and-turn recovery. All corridor decisions come from current lidar and pose data; no obstacle placement is encoded.

## Motion and safety limits

Default Catcher limits are:

| Setting | Value |
|---|---:|
| Cruise linear speed | 0.44 m/s |
| Maximum boost speed | 0.70 m/s |
| Maximum angular speed | 1.8 rad/s |
| Linear acceleration | 1.2 m/s² |
| Angular acceleration | 2.8 rad/s² |
| Prediction horizon | 4.0 s |
| Prediction step | 0.2 s |
| Capture-control distance | 0.55 m |
| Capture radius | 0.50 m |
| Capture hold | 1.0 s |
| Anti-shield trigger | 1.65 m |
| Anti-shield orbit radius | 1.12 m |
| Anti-shield angular step | 0.72 rad |
| Anti-shield dwell (before committing) | 0.35 s |
| Anti-shield max commitment duration | 3.0 s |
| Anti-shield cooldown after giving up | 1.5 s |
| Prediction confidence floor | 0.25 × nominal horizon |
| Intercept-trust switch thresholds | high 0.60 / low 0.30 |
| Obstacle-map resolution | 0.15 m |
| Lidar mapping range | 4.5 m |
| Unobserved-cell upper TTL | 15.0 s (free rays clear sooner) |

The Catcher selects speed continuously from distance: it cruises near the Runner for a stable capture hold and smoothly increases toward boost across longer open approaches. Target-heading error and live lidar corridor clearance reduce that request before the acceleration and hard-limit governor runs.

Non-finite commands are rejected, velocity is clamped, and acceleration is rate-limited. Stale pose data or missing required sensors produces a zero command. A final pose-based boundary governor overrides pursuit and drives inward whenever the Catcher crosses the arena safety line.

## Modes

| Mode | Meaning |
|---|---|
| `INTERCEPT` | Curvature-aware open-space interception (forecast currently trusted). |
| `FLANK` | Persistent route around the Runner's shield obstacle, once the dwell gate is satisfied. |
| `CAPTURE` | Close-range control for completing the legal hold. |
| `CHASE` | Direct pursuit at the Runner's current position -- used by the baseline strategy, and by `predictive`/`aggressive` whenever self-arbitration currently distrusts the forecast. |
| `PRESSURE` | Higher-resolution prediction used by the aggressive test strategy (still subject to the same feasibility check). |
| `*/GAP` | Adaptive navigator is following a selected open corridor. |
| `*/RECOVERY` | Stall watchdog is reversing and changing its preferred side. |
| `BOUNDARY_RETURN` | Final arena governor is driving inward. |
| `SEARCH` | Pose data is stale; velocity is zero. |
| `SENSOR_FAULT` | A required sensor is missing; velocity is zero. |

## Validated result

Historical regression trials established curvature-aware capture and anti-shield flanking. The last full-sensor Gazebo gate used random arena seed 909: the aggressive Catcher faced the unchanged canonical `competitive` Runner, completed a legal capture at 15.099 seconds, reached 0.352 m minimum separation, and recorded zero collisions. The seed, strategies, result JSON, and sensor profile must accompany future performance claims.

**This Gazebo number predates the turn-consistency-gated horizon, feasibility self-arbitration, and dwell-gated flanking described above** -- those were validated on the pure-Python kinematic benchmark only (see the table below and `SIMULATION_AND_OPERATIONS.md`), not yet re-run in Gazebo. Re-run this gate before relying on the 15.099 s figure again.

| Kinematic benchmark (10 seeds, 180 s, 8 Runner behaviors) | Before these changes | After |
|---|---:|---:|
| Overall mean capture time, `predictive` | 13.72 s | 12.75 s |
| `strategic`/`wall_aware`/`obstacle_weaving`, `predictive` | 20.17 s | 18.31 s |
| `adversarial`, `predictive` | 22.33 s | 20.21 s |
| `circular`, `predictive` (the original P0 fix) | 5.20 s | 5.20 s (unchanged) |

`baseline` sits at 17.29 s / 11.09 s / 5.90 s on those same three rows -- the gap to `predictive` narrowed substantially but is not fully closed on `strategic`-family and `adversarial`. Full method and reasoning: see the repository's commit history for `catcher/strategy.py`, `tracking/velocity.py`, and `planning/interception.py`.

The Gazebo result is stored locally at `/tmp/turtle_pursuit_canonical_duel_antishield.json`. It is a deterministic regression result for the current arena and seed, not a mathematical guarantee for unmodeled hardware disturbances or a different official arena.

## Implementation references

- Strategy: [`catcher/strategy.py`](../src/turtle_pursuit/turtle_pursuit/catcher/strategy.py)
- ROS controller: [`catcher/node.py`](../src/turtle_pursuit/turtle_pursuit/catcher/node.py)
- Interception model: [`planning/interception.py`](../src/turtle_pursuit/turtle_pursuit/planning/interception.py)
- Velocity estimation: [`tracking/velocity.py`](../src/turtle_pursuit/turtle_pursuit/tracking/velocity.py)
- Motion and lidar safety: [`control/motion.py`](../src/turtle_pursuit/turtle_pursuit/control/motion.py)
- Dynamic obstacle mapping: [`perception/obstacles.py`](../src/turtle_pursuit/turtle_pursuit/perception/obstacles.py)
- Parameters: [`config/pursuit.yaml`](../src/turtle_pursuit/config/pursuit.yaml)
