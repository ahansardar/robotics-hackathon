# Turtle Pursuit

A ROS 2 Jazzy pursuit-and-evasion product for two TurtleBot 4 Lite robots in one Gazebo Sim 8 world. `catcher` uses direct or predictive pursuit; `runner` uses baseline or strategic evasion; an evaluator enforces the continuous capture hold and records match metrics.

Detailed role documentation:

- [Runner algorithm](docs/RUNNER_ALGORITHM.md)
- [Catcher algorithm](docs/CATCHER_ALGORITHM.md)
- [Simulator, UI, sensors, random arenas, and operations](docs/SIMULATION_AND_OPERATIONS.md)
- [Rule coverage, assumptions, and unresolved organizer details](docs/RULE_COMPLIANCE.md)
- [Competition strategy and validation gates](../WINNING_STRATEGY.md)

## Start here

```bash
cd /home/ahan-sardar/robotics-hackathon/turtle_pursuit_ws
./run_gui.sh round1   # qualification: Catcher vs standardized moving target
./run_gui.sh round2   # honest best-vs-best: predictive Catcher vs competitive Runner
./run_gui.sh demo     # short stationary-target demonstration
./run_gui.sh catcher-gauntlet # Catcher validation vs the canonical competitive Runner
./run_gui.sh runner-gauntlet  # our Runner vs maximum-pressure Catcher
./run_gui.sh hardest          # hardest model vs hardest model
./scripts/preflight.sh # verify the machine and workspace without launching
# In another terminal after controllers start:
./scripts/verify_live_sensors.sh full
```

Every `run_gui.sh` invocation generates a new collision-free four-obstacle layout and prints its seed and poses before launch. Set `TURTLE_PURSUIT_ARENA_SEED=404` to replay an exact arena. The telemetry UI opens a separate 60 Hz sensor window with both robots' RGB images, depth images, lidar point views, measured input FPS, and currently mapped obstacle centers. The full profile configures each simulated RGB-D camera for 60 Hz; the delivered rate still depends on the host GPU and Gazebo real-time factor.

Run one command at a time. Red is the Catcher, blue is the Runner, and the blue disk is the 0.5 m capture zone. The match begins after robot setup and lasts 180 seconds. Press `Ctrl+C` in the terminal to stop.

GUI runs open the Gazebo arena and a separate live telemetry dashboard. The panel shows elapsed/remaining time, current and minimum separation, continuous capture-hold progress, both speeds and path lengths, collisions, controller modes, and live lidar/RGB-D health. Gauntlet opponents are deterministic for a fixed seed, so captures and survivals can be compared fairly between code changes.

There is one canonical competition Runner: `competitive`. It reverses immediately when appropriate, commits to obstacle shielding, and only changes cover after it is genuinely lost. Both roles build independent ray-cleared, time-aware obstacle maps from lidar; no obstacle coordinates are configured. Empty lidar rays immediately erase relocated geometry, while a bounded history combines multiple viewpoints into better obstacle-center estimates. The predictive Catcher detects the shielding equilibrium, commits to an obstacle flank, and wins position through interception rather than a higher configured top speed. The current equal-speed release was checked in full-sensor mode on random arena seed 909: aggressive Catcher versus the unchanged competitive Runner produced a legal capture at 15.099 s, 0.352 m minimum separation, and zero collisions. This is reproducible regression evidence, not a mathematical guarantee against changes outside sensor range or during sensor latency.

Every GUI run now executes a fail-fast preflight first. It checks ROS 2, required TurtleBot/Gazebo packages, the built overlay, stale source changes, and an already-running Gazebo process. A failed check prints the exact corrective action and launches nothing.

The supplied event document does not define an official technical simulator, exact obstacle layout, ROS/Gazebo release, or sensor interface. This project implements its stated provisional format: 10×10 m enclosed arena, four static obstacles, 3 m initial separation, simultaneous autonomous start, three-minute inning, and a continuous one-second hold within 0.5 m. Round 1 uses a fixed deterministic moving TurtleBot because the document permits an organizer-controlled TurtleBot target.

## Architecture

The launch layer starts one Gazebo server and inserts two locally generated TurtleBot 4 Lite descriptions serially. Gazebo owns exactly one world-level sensor system. Each robot keeps one RPLidar and one RGB-D camera; the eleven redundant GPU cliff/IR rays from the installed model are removed because two copies overload Gazebo 8's render thread. Bumper/hazard, IMU description, pose, wheel control, lidar, RGB, depth, links, and visuals remain. A trimmed bridge starts only competition-critical processes. Each robot has a namespace and Gazebo entity (`catcher/turtlebot4`, `runner/turtlebot4`).

Three sensor profiles are available: `full` (default: RPLidar + RGB-D), `lidar` (RPLidar without camera rendering), and `stable` (no render-backed sensors, emergency compatibility mode). `run_gui.sh` uses `full`; override at the end of the command with `sensors:=lidar` or `sensors:=stable`.

The installed Jazzy stack was inspected locally. Its actual interfaces used here are:

| Purpose | Catcher | Runner | Type |
|---|---|---|---|
| global simulator state | `/catcher/sim_ground_truth_pose` | `/runner/sim_ground_truth_pose` | `nav_msgs/Odometry` (best effort) |
| velocity command | `/catcher/diffdrive_controller/cmd_vel` | `/runner/diffdrive_controller/cmd_vel` | `geometry_msgs/TwistStamped` |
| lidar obstacle input | `/catcher/scan` | `/runner/scan` | `sensor_msgs/LaserScan` |
| RGB camera | `/catcher/camera/color/image_raw` | `/runner/camera/color/image_raw` | `sensor_msgs/Image` |
| depth camera | `/catcher/camera/depth/image_raw` | `/runner/camera/depth/image_raw` | `sensor_msgs/Image` |
| collision hazards | `/catcher/hazard_detection` | `/runner/hazard_detection` | `irobot_create_msgs/HazardDetectionVector` |

Project outputs are `/match/state` (JSON in `std_msgs/String`), `/match/catcher_mode`, `/match/runner_mode`, and `/match/markers`. The marker array shows both positions and the Runner's capture radius. The simulation works with RViz disabled.

## Prerequisites and build

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8, and the TurtleBot packages named in the challenge brief must be installed under `/opt/ros/jazzy`.

```bash
cd /home/ahan-sardar/robotics-hackathon/turtle_pursuit_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## Run a match

```bash
cd /home/ahan-sardar/robotics-hackathon/turtle_pursuit_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtle_pursuit pursuit.launch.py \
  headless:=true rviz:=false sensors:=full world:=pursuit_arena seed:=1 \
  match_duration:=180.0 catcher_strategy:=predictive runner_strategy:=strategic
```

The robots are inserted roughly 3 m apart in a local 10×10 m, four-obstacle arena. Entity insertion is serialized, but both autonomous controllers and the match timer start together after `startup_delay` (25 seconds by default). No docks are spawned. Use `headless:=false rviz:=true` for visual debugging. `world:=warehouse` remains available. Result JSON defaults to `/tmp/turtle_pursuit_result.json`; override with `result_file:=/absolute/path.json`.

Strategies are `baseline`, `predictive`, or `aggressive` for Catcher and `baseline`, `strategic`, `adversarial`, `competitive`, `standardized`, or `stationary` for Runner. `aggressive` increases curvature-aware search resolution and prediction pressure. `adversarial` evaluates twice as many headings and alternates deterministic perpendicular breaks to defeat constant-motion prediction. `competitive` uses immediate bidirectional escape and persistent obstacle shielding. `stationary` is only a validation behaviour.

Both roles now have the same default boost ceiling: `catcher_max_linear:=0.70` and `runner_max_linear:=0.70`. These are simulator settings, not limits claimed by the supplied event brief, and either can be overridden when the organizer publishes an official limit. Speed is continuously selected rather than held at the ceiling: the Runner moves from a 0.44 m/s cruise toward boost as the Catcher closes inside 3.0 m, while the Catcher boosts over long open approaches and returns toward cruise near the Runner for capture control. Heading error, live lidar clearance, boundary recovery, acceleration limits, and the final velocity clamp can all reduce the requested speed.

## Strategies and safety

Predictive pursuit estimates target velocity and turn rate, projects curved motion, selects the earliest speed-feasible intercept, and keeps intercept guidance active until capture. When the Runner shields behind a lidar-mapped obstacle, the Catcher enters `FLANK`, commits to one orbit direction, and cuts around the obstacle rather than settling opposite the Runner. Both roles pass their tactical goal through an adaptive lidar navigator: it evaluates 48 possible body-width corridors, balances goal progress against clearance, retains its chosen side, and reverses/reorients after detecting a stall. Modes expose this as `/GAP` or `/RECOVERY`. During final approach the Catcher distinguishes the Runner surface from a wall so safety does not block legal capture. RGB-D color/depth tracks the red or blue role marker and can restore the opponent pose if the primary pose stream becomes stale.

Both robots use acceleration/velocity limiting, finite-command rejection, fresh-sensor gates, stale-state stops, shutdown zero commands, and a final pose-based `BOUNDARY_RETURN` governor that overrides tactics at the arena safety line. In `full` mode, loss of lidar or camera input changes the controller mode to `SENSOR_FAULT` and commands zero velocity. Lidar returns are world-projected into a persistent 0.15 m occupancy grid; free-space rays clear stale cells, walls and the tracked opponent are filtered, and compact obstacle components are supplied to `SHIELD` and `FLANK`. Strategic evasion scores escape headings using Catcher distance, wall/open-region clearance, corner avoidance, and smoothness; inside 1.15 m it performs a high-priority perpendicular `BREAKAWAY`.

Capture requires separation `<= 0.5 m` continuously for `1.0 s`. Leaving the radius resets the hold timer. The evaluator also records minimum separation, path lengths, bump/stall collision transitions, modes, capture/survival time, and stops both robots on completion.

## Configuration

Edit `src/turtle_pursuit/config/pursuit.yaml`, rebuild, and source the overlay. Important knobs are `max_linear`, `cruise_linear`, `runner_boost_distance`, `catcher_cruise_distance`, `catcher_boost_distance`, acceleration limits, `prediction_horizon`, `prediction_step`, `velocity_alpha`, `capture_control_distance`, `capture_speed`, `boundary_margin`, Runner score weights, `stale_timeout`, `capture_radius`, `capture_hold`, and `match_duration`. Equal ceilings make the contest depend on interception and evasion rather than a baked-in speed advantage. Increase clearance weights if the Runner enters corners; reduce velocity alpha for smoother but slower estimates.

Pose, command, scan, RGB, depth, and camera-info topic names are configuration values. Override the `*_topic` parameters in `pursuit.yaml` when the organizer publishes its technical interface; no strategy source edit is required.

## Benchmarks

The deterministic kinematic benchmark compares both Catcher strategies across stationary, straight, random-walk, circular, wall-aware, obstacle-weaving, strategic, and adversarial behaviours. The wrapper exits nonzero if any capture is missed, so it can be used as a release/CI gate:

```bash
cd /home/ahan-sardar/robotics-hackathon/turtle_pursuit_ws
bash scripts/run_benchmarks.sh 10 180
# or: ros2 run turtle_pursuit benchmark --seeds 10 --duration 180 --output benchmark_results.csv
```

CSV columns contain strategies, seed, capture success/time, survival time, minimum separation, collisions, and both path lengths. This is an algorithm benchmark, not a substitute for the Gazebo trial.

The committed equal-speed benchmark was regenerated with 10 seeds, 180-second limits, eight Runner behaviors, and both Catcher strategies: 160/160 cases captured. Mean capture time was 11.15 s for baseline and 13.72 s for predictive. These kinematic results validate termination and regression coverage; they do not rank the strategies under physical obstacles and sensor latency.

## Extending the adapter

Implement the same methods as `RosStateAdapter`: `get_catcher_pose`, `get_runner_pose`, both velocity getters, `get_obstacles`, both send methods, and `stale`. Convert sensor timestamps into the node's clock domain and preserve the internal dataclasses. Then instantiate the new adapter in the three nodes. This isolates future competition pose, odometry, camera, or perception interfaces from planning.

## Troubleshooting

- If commands do not connect, verify `ros2 topic info /catcher/diffdrive_controller/cmd_vel -v`; the controller consumes `TwistStamped`, not `Twist`.
- Ground-truth topics are best-effort. Reliable subscriptions receive nothing; the adapter uses the sensor-data QoS profile.
- If full rendering is unavailable on the match machine, retry with `sensors:=lidar`; use `sensors:=stable` only as the last-resort compatibility profile.
- After startup, run `./scripts/verify_live_sensors.sh full` in another terminal. Any missing competition-critical stream causes a nonzero exit.
- Do not set `startup_delay` below the time required for both namespaced diff-drive controllers to activate (about 20 seconds on the validation machine).
- Warehouse is much larger than the provisional 10 m arena. Boundary scoring uses a configurable 5 m half-width around the origin; replace the world/map when official geometry arrives.

## Requirement coverage and limitations

Implemented: one-world/two-namespace launch, autonomous independent commands, replaceable state adapter, two Catcher and two Runner strategies, curvature-aware interception, deterministic A*, live lidar safety, RGB-D fallback tracking, sensor watchdogs, continuous capture evaluation, collision/path metrics, JSON/CSV results, seeded benchmark gates, markers, YAML tuning, three sensor profiles, and automated algorithm/ROS/profile tests.

The primary simulator pose adapter still uses Gazebo ground truth because the official technical interface has not been released; RGB-D is a tested fallback, not yet the sole localization source. A* is tested but is not fed the live occupancy map; local RPLidar navigation and obstacle mapping are active. Predicted path/escape markers are not all separately rendered. Official sensor restrictions and submission interfaces remain rulebook-dependent. The latest two-robot `full`-profile headless physics gate used random seed 909 and equal 0.70 m/s ceilings: capture at 15.099 s, 0.352 m minimum separation, and zero collisions.
