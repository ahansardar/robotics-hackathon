# Simulator and Operations Guide

This document describes the simulator implemented in this repository. It distinguishes measured behavior from competition rules; see [RULE_COMPLIANCE.md](RULE_COMPLIANCE.md) before treating any simulator setting as official.

## System layout

One Gazebo Sim 8 process hosts two TurtleBot 4 Lite entities. Each robot has an independent ROS namespace, controller, lidar, RGB-D camera, hazard stream, pose stream, obstacle map, and velocity output. Red is the Catcher and blue is the Runner.

| Component | Responsibility |
|---|---|
| `pursuit.launch.py` | World, robot insertion, bridges, controllers, evaluator, and optional dashboard |
| `sensorless_spawn.launch.py` | Generates the role-specific TurtleBot description and inserts it serially |
| `essential_bridge.launch.py` | Bridges only the simulator topics required by the match |
| `RosStateAdapter` | Converts ROS messages into controller state and publishes bounded commands |
| `ObstacleMapper` | Builds and clears a local world-frame obstacle occupancy history from lidar |
| `AdaptiveNavigator` | Selects a collision-free body-width corridor and performs stall recovery |
| evaluator | Applies the capture rule and publishes JSON telemetry and RViz markers |
| dashboard | Displays match metrics and live RGB, depth, lidar, map, and measured FPS |

## Reproducible launch modes

Build once, then use the wrapper:

```bash
cd /home/ahan-sardar/robotics-hackathon/turtle_pursuit_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
./run_gui.sh round1
```

| Mode | Catcher | Runner | Intended use |
|---|---|---|---|
| `round1` | predictive | standardized | qualification rehearsal |
| `round2` | predictive | competitive | our best-vs-best comparison |
| `demo` | predictive | stationary | short pipeline demonstration |
| `catcher-gauntlet` | predictive | competitive | Catcher regression against the canonical Runner |
| `runner-gauntlet` | aggressive | competitive | Runner regression against maximum pressure |
| `hardest` | aggressive | competitive | hardest configured matchup |

Arguments after the mode are forwarded to the ROS launch file. For example:

```bash
./run_gui.sh hardest match_duration:=180.0 sensors:=full
```

## Random obstacle layouts

Unless `world:=...` is supplied, `run_gui.sh` generates a new four-obstacle arena in `/tmp`. The generator samples positions and yaw angles inside the arena, preserves clearance around both start poses, and enforces spacing between obstacles. Controllers receive no generated coordinates.

Set a seed to reproduce a failure exactly:

```bash
TURTLE_PURSUIT_ARENA_SEED=505 ./run_gui.sh hardest
```

The seed and sampled poses are printed before Gazebo starts. Supplying a world explicitly disables generation:

```bash
./run_gui.sh hardest world:=pursuit_shifted world_name:=pursuit_shifted
```

## Sensor profiles

| Profile | Lidar | RGB-D | Required-controller behavior |
|---|---:|---:|---|
| `full` | yes | yes | stop with `SENSOR_FAULT` if a required stream becomes stale |
| `lidar` | yes | no | require lidar; avoid camera rendering |
| `stable` | no | no | compatibility mode using pose and hazard interfaces |

The full profile requests 60 Hz RGB-D updates. The dashboard reports measured input rate; 60 FPS is a target and is not guaranteed when Gazebo's real-time factor or GPU rendering falls behind.

During a running match, verify every required stream:

```bash
./scripts/verify_live_sensors.sh full
```

The script exits nonzero if controller modes, both scans, or any required RGB/depth stream fails to publish before the deadline.

## Dynamic motion

Both roles use an equal default maximum linear command of 0.70 m/s and a 0.44 m/s cruise speed. These are simulator parameters, not an asserted competition limit.

The Runner smoothly raises requested speed as separation falls inside its 3.0 m threat range. The Catcher raises requested speed over long approaches and returns toward cruise near the Runner to stabilize the continuous capture hold. The request then passes through, in order:

1. target-heading alignment;
2. current lidar corridor clearance;
3. boundary recovery override;
4. finite-value rejection;
5. acceleration and angular-rate limiting;
6. the configured hard command ceiling.

Consequently, neither robot drives at one fixed speed and neither receives a default top-speed advantage.

## UI and telemetry

The dashboard shows elapsed and remaining time, current and minimum separation, capture-hold progress, both measured speeds, both path lengths, collision count, controller modes, sensor freshness, measured camera FPS, RGB images, depth images, lidar point views, and mapped obstacle centers.

`/match/state` contains the machine-readable JSON record. The evaluator writes final JSON to `/tmp/turtle_pursuit_result.json` by default. Override it with `result_file:=/absolute/path.json`.

## Verification workflow

```bash
./scripts/preflight.sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtle_pursuit --symlink-install
colcon test --packages-select turtle_pursuit --event-handlers console_direct+
colcon test-result --verbose
bash scripts/run_benchmarks.sh 10 180
```

Before accepting a controller change, also run at least one full-sensor Gazebo trial in a new random arena and replay any previously failing seed. Pure-Python benchmarks do not model wheel dynamics, render load, sensor latency, or obstacle collisions. They do measure Catcher/Runner body-contact transitions (separation crossing the combined ~0.34 m footprint radius) as the `collisions` benchmark column, which is a real signal, not a placeholder -- but it is still not a substitute for physical contact validated in Gazebo.

Current release evidence:

| Gate | Configuration | Result |
|---|---|---|
| unit tests (pure Python, no ROS) | `test_algorithms.py` + `test_sensorless_description.py` | 45 passed |
| ROS smoke test | requires `rclpy`, not independently re-run outside a ROS environment | last known: passed; unaffected by the algorithm changes below |
| kinematic benchmark | 10 seeds × 8 Runner behaviors × 2 Catchers, 180 s | 160/160 captures, mean capture 11.32 s (baseline) / 12.75 s (predictive) |
| Gazebo physics | seed 909, full sensors, aggressive vs competitive, equal 0.70 m/s ceilings | captured at 15.099 s; 0.352 m minimum; 0 collisions -- **predates** the turn-consistency/self-arbitration/flank-dwell changes; re-run before relying on this number |

## Known limitations

- Global pose currently comes from simulator odometry because the organizer's final interface is unknown. RGB-D target tracking is a fallback, not the only localization source.
- Obstacles are static during an inning, matching the supplied rules. Their positions change between generated runs; lidar clearing also supports relocated geometry during development tests.
- No finite test suite proves success for every physically possible layout. Generator spacing, robot kinematics, sensor range, and arena reachability define the supported domain.
- `stable` mode is a compatibility fallback and does not satisfy a requirement to use lidar and camera continuously.
