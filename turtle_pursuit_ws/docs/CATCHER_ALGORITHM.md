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

## Open-space interception

The predictor samples future Runner positions from the present to the configured horizon. It selects the earliest position satisfying:

```text
distance(Catcher, predicted Runner) / Catcher speed ≤ prediction time + tolerance
```

The default predictive horizon is 4.0 seconds with a 0.2-second step. The target is replanned every control tick, so it adapts continuously rather than committing to a stale intercept.

## Anti-shield flanking

Direct interception cannot defeat persistent obstacle shielding. It creates an equilibrium: the Catcher approaches one side while the Runner remains opposite the obstacle. The Catcher therefore detects when the Runner is within 1.65 m of a lidar-mapped obstacle and switches to `FLANK`.

The Catcher builds its own map; it does not receive the Runner's map or depend on cooperation from an opponent. Lidar hits are accumulated in a 0.15 m time-aware world grid, arena walls and the tracked Runner are filtered out, and compact connected components provide obstacle centers. Each beam clears free grid cells before its return, so moved or removed obstacles are erased as soon as lidar sees through their old location. A bounded 15-second history combines surfaces seen from different viewpoints for more accurate centers.

The flank planner:

1. Identifies the obstacle nearest the Runner.
2. Measures the Catcher and Runner angles around that obstacle.
3. Selects a clockwise or counter-clockwise direction using observed Runner angular motion, falling back to the shortest angular route.
4. Retains that direction while the Runner uses the same obstacle.
5. Places the next target 0.72 rad ahead on a 1.12 m orbit.

Direction persistence prevents oscillation. Instead of pointing through the obstacle, the Catcher advances around it and uses its higher maximum speed to reduce angular separation. If the Runner abandons that obstacle, normal interception resumes or a new flank is initialized.

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
| Obstacle-map resolution | 0.15 m |

The Catcher selects speed continuously from distance: it cruises near the Runner for a stable capture hold and smoothly increases toward boost across longer open approaches. Target-heading error and live lidar corridor clearance reduce that request before the acceleration and hard-limit governor runs.
| Lidar mapping range | 4.5 m |
| Unobserved-cell upper TTL | 15.0 s (free rays clear sooner) |

Non-finite commands are rejected, velocity is clamped, and acceleration is rate-limited. Stale pose data or missing required sensors produces a zero command. A final pose-based boundary governor overrides pursuit and drives inward whenever the Catcher crosses the arena safety line.

## Modes

| Mode | Meaning |
|---|---|
| `INTERCEPT` | Curvature-aware open-space interception. |
| `FLANK` | Persistent route around the Runner's shield obstacle. |
| `CAPTURE` | Close-range control for completing the legal hold. |
| `CHASE` | Direct pursuit used only by the baseline strategy. |
| `PRESSURE` | Higher-resolution prediction used by the aggressive test strategy. |
| `*/GAP` | Adaptive navigator is following a selected open corridor. |
| `*/RECOVERY` | Stall watchdog is reversing and changing its preferred side. |
| `BOUNDARY_RETURN` | Final arena governor is driving inward. |
| `SEARCH` | Pose data is stale; velocity is zero. |
| `SENSOR_FAULT` | A required sensor is missing; velocity is zero. |

## Validated result

Historical regression trials established curvature-aware capture and anti-shield flanking. The current equal-speed release gate used full sensors and random arena seed 909. The aggressive Catcher faced the unchanged canonical `competitive` Runner, completed a legal capture at 15.099 seconds, reached 0.352 m minimum separation, and recorded zero collisions. The seed, strategies, result JSON, and sensor profile must accompany future performance claims.

The result is stored locally at `/tmp/turtle_pursuit_canonical_duel_antishield.json`. It is a deterministic regression result for the current arena and seed, not a mathematical guarantee for unmodeled hardware disturbances or a different official arena.

## Implementation references

- Strategy: [`catcher/strategy.py`](../src/turtle_pursuit/turtle_pursuit/catcher/strategy.py)
- ROS controller: [`catcher/node.py`](../src/turtle_pursuit/turtle_pursuit/catcher/node.py)
- Interception model: [`planning/interception.py`](../src/turtle_pursuit/turtle_pursuit/planning/interception.py)
- Velocity estimation: [`tracking/velocity.py`](../src/turtle_pursuit/turtle_pursuit/tracking/velocity.py)
- Motion and lidar safety: [`control/motion.py`](../src/turtle_pursuit/turtle_pursuit/control/motion.py)
- Dynamic obstacle mapping: [`perception/obstacles.py`](../src/turtle_pursuit/turtle_pursuit/perception/obstacles.py)
- Parameters: [`config/pursuit.yaml`](../src/turtle_pursuit/config/pursuit.yaml)
