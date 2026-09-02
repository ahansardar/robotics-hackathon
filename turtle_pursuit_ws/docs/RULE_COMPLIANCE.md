# Rule Coverage and Assumptions

The repository contains two source documents: the root [RULEBOOK.md](../../RULEBOOK.md) and the supplied event brief, [All that you need to know about TurtleBo.md](../All%20that%20you%20need%20to%20know%20about%20TurtleBo.md). This page records how the implementation maps to them without presenting project choices as organizer rules.

## Implemented rule mapping

| Requirement | Implementation |
|---|---|
| 10 m × 10 m enclosed arena | `pursuit_arena.sdf` uses a bounded 10 m square |
| 4–6 static obstacles | default and generated arenas contain four static obstacles |
| 3 m initial separation | Catcher starts at x = -1.5 m and Runner at x = +1.5 m |
| no head start | both controllers and evaluator are started together after robot setup |
| maximum three-minute inning | evaluator default is 180 seconds |
| capture at distance ≤ 0.5 m | `CaptureDetector` uses a 0.50 m radius |
| continuously held for one second | leaving the radius resets the hold timer; default hold is 1.0 second |
| autonomous Catcher and Runner | independent ROS nodes publish independent velocity commands |
| obstacle avoidance and adaptation | each role consumes its own lidar and builds its own obstacle map |
| real-time response | strategies, navigation, and speed are recomputed at the configured control rate |
| Round 1 standardized target | `standardized` Runner provides a deterministic rehearsal target |
| GitHub documentation | README plus separate role, simulator, compliance, and strategy documents |

## Not stated by the supplied rules

The documents do not specify a maximum linear speed, acceleration limit, maximum angular speed, exact obstacle coordinates, exact TurtleBot configuration, ROS distribution, Gazebo version, topic names, permitted sensor list, or whether simulator ground-truth pose is allowed. The current values are therefore replaceable engineering assumptions:

- both roles use the same 0.70 m/s command ceiling;
- ROS 2 Jazzy and Gazebo Sim 8 are used locally;
- TurtleBot 4 Lite is used because the platform is described as tentative;
- lidar, RGB-D, hazard data, and simulator odometry are enabled;
- generated layouts use four obstacles, within the stated 4–6 range.

## Shielding

`SHIELD` is the name of a navigation tactic, not a physical shield, added object, collision modification, or hidden sensor. The Runner uses lidar-detected static obstacles as legal cover and travels around them. Nothing in the supplied documents prohibits this behavior; final legality remains subject to the organizer's technical rulebook and interpretation.

## Items requiring organizer confirmation

Before final submission, confirm:

1. official simulator, ROS version, and robot description;
2. allowed sensors and whether ground-truth pose topics are permitted;
3. official command topics and message types;
4. speed, acceleration, and angular-rate constraints;
5. obstacle geometry, count, placement policy, and whether layouts change between innings;
6. standardized Round 1 target interface and motion profile;
7. collision penalties, boundary-contact policy, and reset behavior;
8. submission packaging and launch-command requirements.

When official details arrive, update parameters and the ROS adapter first. Do not encode arena coordinates or opponent motion into either competition strategy.
