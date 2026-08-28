# Turtle Pursuit

A ROS 2 Jazzy pursuit-and-evasion product for two TurtleBot 4 Lite robots in one Gazebo Sim 8 world. `catcher` uses direct or predictive pursuit; `runner` uses baseline or strategic evasion; an evaluator enforces the continuous capture hold and records match metrics.

## Start here

```bash
cd /home/ahan-sardar/robotics-hackathon/turtle_pursuit_ws
./run_gui.sh round1   # qualification: Catcher vs standardized moving target
./run_gui.sh round2   # knockout: autonomous Catcher vs autonomous Runner
./run_gui.sh demo     # short stationary-target demonstration
```

Run one command at a time. Red is the Catcher, blue is the Runner, and the blue disk is the 0.5 m capture zone. The match begins after robot setup and lasts 180 seconds. Press `Ctrl+C` in the terminal to stop.

The supplied event document does not define an official technical simulator, exact obstacle layout, ROS/Gazebo release, or sensor interface. This project implements its stated provisional format: 10×10 m enclosed arena, four static obstacles, 3 m initial separation, simultaneous autonomous start, three-minute inning, and a continuous one-second hold within 0.5 m. Round 1 uses a fixed deterministic moving TurtleBot because the document permits an organizer-controlled TurtleBot target.

## Architecture

The launch layer starts one Gazebo server and invokes a local sensorless TurtleBot spawn integration twice, serially. The local description is generated from the installed TurtleBot 4 Lite xacro, then removes render-backed GPU sensors and the duplicate per-model sensor system that crashes Gazebo 8 with two robots. Wheel physics, `gz_ros2_control`, contacts, IMU, pose publication, links, and visuals remain. Each robot has a namespace and Gazebo entity (`catcher/turtlebot4`, `runner/turtlebot4`). The replaceable adapter converts simulator messages into stable internal types and bounded commands.

The installed Jazzy stack was inspected locally. Its actual interfaces used here are:

| Purpose | Catcher | Runner | Type |
|---|---|---|---|
| global simulator state | `/catcher/sim_ground_truth_pose` | `/runner/sim_ground_truth_pose` | `nav_msgs/Odometry` (best effort) |
| velocity command | `/catcher/diffdrive_controller/cmd_vel` | `/runner/diffdrive_controller/cmd_vel` | `geometry_msgs/TwistStamped` |
| optional lidar adapter input | `/catcher/scan` | `/runner/scan` | `sensor_msgs/LaserScan` |
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
  headless:=true rviz:=false world:=pursuit_arena seed:=1 \
  match_duration:=180.0 catcher_strategy:=predictive runner_strategy:=strategic
```

The robots are inserted roughly 3 m apart in a local 10×10 m, four-obstacle arena. Entity insertion is serialized, but both autonomous controllers and the match timer start together after `startup_delay` (25 seconds by default). No docks are spawned. Use `headless:=false rviz:=true` for visual debugging. `world:=warehouse` remains available. Result JSON defaults to `/tmp/turtle_pursuit_result.json`; override with `result_file:=/absolute/path.json`.

Strategies are `baseline` or `predictive` for Catcher and `baseline`, `strategic`, or `stationary` for Runner. `stationary` is a validation behaviour.

## Strategies and safety

Predictive pursuit smooths velocity observations, samples future Runner positions, selects the earliest speed-feasible intercept, replans continuously, transitions through SEARCH/INTERCEPT/CHASE/CAPTURE, and uses a slow capture controller near 0.5 m. Both robots use acceleration/velocity limiting, finite-command validation, stale-state stops, and shutdown zero commands. Strategic evasion scores eleven escape headings using Catcher distance, wall/open-region clearance, corner avoidance, and heading smoothness with seeded jitter. Deterministic A* and reactive scan avoidance are implemented, but the stable sensorless launch does not publish scans; connect a competition sensor adapter or a CPU-safe lidar before obstacle-heavy live matches.

Capture requires separation `<= 0.5 m` continuously for `1.0 s`. Leaving the radius resets the hold timer. The evaluator also records minimum separation, path lengths, bump/stall collision transitions, modes, capture/survival time, and stops both robots on completion.

## Configuration

Edit `src/turtle_pursuit/config/pursuit.yaml`, rebuild, and source the overlay. Important knobs are `max_linear`, acceleration limits, `prediction_horizon`, `prediction_step`, `velocity_alpha`, `chase_distance`, `capture_control_distance`, `capture_speed`, `boundary_margin`, Runner score weights, `stale_timeout`, `capture_radius`, `capture_hold`, and `match_duration`. Keep Catcher speed above Runner speed for feasible interception. Increase clearance weights if the Runner enters corners; reduce velocity alpha for smoother but slower estimates.

## Benchmarks

The deterministic kinematic benchmark compares both Catcher strategies across stationary, straight, random-walk, circular, wall-aware, obstacle-weaving, and strategic behaviours:

```bash
cd /home/ahan-sardar/robotics-hackathon/turtle_pursuit_ws
bash scripts/run_benchmarks.sh 10 180
# or: ros2 run turtle_pursuit benchmark --seeds 10 --duration 180 --output benchmark_results.csv
```

CSV columns contain strategies, seed, capture success/time, survival time, minimum separation, collisions, and both path lengths. This is an algorithm benchmark, not a substitute for the Gazebo trial.

## Extending the adapter

Implement the same methods as `RosStateAdapter`: `get_catcher_pose`, `get_runner_pose`, both velocity getters, `get_obstacles`, both send methods, and `stale`. Convert sensor timestamps into the node's clock domain and preserve the internal dataclasses. Then instantiate the new adapter in the three nodes. This isolates future competition pose, odometry, camera, or perception interfaces from planning.

## Troubleshooting

- If commands do not connect, verify `ros2 topic info /catcher/diffdrive_controller/cmd_vel -v`; the controller consumes `TwistStamped`, not `Twist`.
- Ground-truth topics are best-effort. Reliable subscriptions receive nothing; the adapter uses the sensor-data QoS profile.
- If Gazebo sensor rendering aborts, confirm `sensorless_spawn.launch.py` is being used and no old `gz sim` process is running.
- Do not set `startup_delay` below the time required for both namespaced diff-drive controllers to activate (about 20 seconds on the validation machine).
- Warehouse is much larger than the provisional 10 m arena. Boundary scoring uses a configurable 5 m half-width around the origin; replace the world/map when official geometry arrives.

## Requirement coverage and limitations

Implemented: one-world/two-namespace launch, autonomous independent commands, replaceable state adapter, two Catcher and two Runner strategies, predictive interception, deterministic A*, optional scan safety, continuous capture evaluator, collision/path metrics, JSON/CSV results, seeded benchmark suite, markers, YAML tuning, a sensorless Gazebo adapter, and automated algorithm/ROS smoke tests.

Current simulator adapter intentionally uses Gazebo ground truth. The stable two-robot launch disables camera, lidar, cliff, and IR rendering because the installed descriptions cause a Gazebo 8 render-thread segmentation fault; therefore live obstacle sensing must be restored through a CPU-safe or competition-provided adapter. A* is tested but is not yet fed a live occupancy map. Predicted intercept/path/escape-target markers are not all separately rendered. Official sensor restrictions and submission interfaces remain rulebook-dependent. A headless physics capture against the stationary Runner was verified on this machine: 9.102 seconds, 0.37 m minimum separation, and 2.63 m Catcher path.
