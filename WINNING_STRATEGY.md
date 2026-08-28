# TurtleBot Pursuit & Evasion — Winning Strategy & Preparation Playbook

**Event:** TurtleBot Pursuit & Evasion Challenge (TechZephyr, IIT Bhubaneswar) — "Chase. Evade. Outthink."
**Prize pool:** ₹25,000
**Today:** 2026-08-28 | **Round 1 (Online):** 12–13 Sep 2026 (~2 weeks out) | **Round 2 (Offline):** 30 Oct – 1 Nov 2026 (~9 weeks out)
**Repo analyzed:** [github.com/ahansardar/robotics-hackathon](https://github.com/ahansardar/robotics-hackathon) — `turtle_pursuit_ws` (ROS 2 Jazzy + Gazebo Sim 8)

This document reconciles both source documents (the formal **Rule Book** and the **"All that you need to know"** brief), audits the existing codebase against them, and lays out a prioritized plan to be competitive in both rounds. It is written to be exhaustive: every rule clause, every code path, and every known unknown is addressed with a concrete action.

---

## 0. Executive Summary

**Where you stand:** the repo is not a toy — it's a working ROS 2 package with two Catcher strategies (baseline, predictive-intercept), three Runner strategies (baseline, strategic, standardized), a live capture evaluator, a deterministic kinematic benchmark harness, and 7 passing unit tests. This is well ahead of "day 1." Treat this plan as *hardening and de-risking* an existing system, not building from scratch.

**Top 3 risks, in order of severity:**

1. **P0 — Confirmed algorithmic blind spot.** Your own benchmark data (`benchmark_results.csv`) shows **both** Catcher strategies fail 100% of the time against a `circular`-moving Runner (0/5 seeds captured in 60s, separation plateaus at ~0.6 m). This is not hypothetical — it's measured. If the Round 1 standardized target moves in a loop/arc (a very plausible choice for a "standardized, reproducible" target), you fail qualification outright. See §4.
2. **P0 — Live perception is currently disabled.** Camera, lidar, cliff, and IR sensors are turned off in the stable two-robot launch because they crash Gazebo 8 with a render-thread segfault. Your A* planner and reactive scan-avoidance are implemented and unit-tested but are **not fed any live obstacle data** in the working configuration. You are currently flying blind around obstacles in the "sensorless" launch. See §7.1.
3. **P1 — The official technical rulebook does not exist yet.** Simulator version, sensor interface, exact obstacle geometry, and the Round 1 target's actual representation/motion are all undefined by the organizers. Your current assumptions (ROS 2 Jazzy, Gazebo Sim 8, ground-truth odometry, 4 obstacles at the corners) are reasonable *guesses*, not guarantees. See §2.

**What this document gives you:** a rules digest, a list of open questions to fire at organizers now, a prioritized engineering backlog, a tuning/benchmarking method, a full test matrix, a risk register, and round-by-round submission and day-of checklists.

---

## 1. Rules Digest (reconciled from both source documents)

### 1.1 Objective
- **Catcher:** locate, pursue, and capture the Runner in the shortest time.
- **Runner:** evade and survive the full inning.

### 1.2 Format
| Round | Mode | What happens |
|---|---|---|
| **1 — Catcher Qualification** | Online, 12–13 Sep | Your Catcher vs. a standardized moving target (organizer-controlled sphere/dot **or** TurtleBot — undefined which). Metric: capture time. Top 8 fastest advance; if <8 qualify, the committee may improvise (e.g., pick top 4 directly). |
| **2 — 1v1 Knockout** | Offline, 30 Oct–1 Nov | Bracket: QF → SF → Final, plus a 3rd-place match between SF losers. Each matchup = 2 innings × 3 min, roles swapped (A catches in inning 1, B catches in inning 2). Both roles scored. |

### 1.3 Arena & match conditions (these numbers are rulebook-canonical — trust them)
| Parameter | Value |
|---|---|
| Arena | 10 m × 10 m, flat, enclosed, 2D |
| Obstacles | 4–6 static |
| Initial separation | 3 m |
| Head start | None — simultaneous autonomous start |
| Max inning duration | 3 minutes (180 s) |

### 1.4 Capture rule (exact wording matters)
> Distance ≤ 0.5 m, held **continuously**, for 1 second.

- A momentary dip ≤0.5 m does **not** count.
- Leaving the radius resets the hold timer to zero (confirmed in your own `CaptureDetector` unit test — this matches the rule precisely).

### 1.5 Win conditions
- **Catcher wins** if capture happens within 3 min; shorter time = better.
- **Runner wins** if the Catcher fails to capture within 3 min; final separation may be used as a tiebreaker.

### 1.6 Tie-breaking order
1. Both capture → faster capture time wins.
2. Both evade → larger final separation wins.
3. One captures, one doesn't → the capturer wins.
4. Still tied → an unspecified "additional standardized tie-breaker" (open question, §2).

### 1.7 Submission
- **Round 1:** GitHub repo with a comprehensive `README.md` **plus video evidence**. No mention of a live/witnessed run — this reads as an asynchronous, self-recorded submission.
- **Round 2:** in-person, offline, head-to-head, 3-minute innings.

### 1.8 Platform
- Simulated, tentatively **TurtleBot 4 Lite**, tentatively **Gazebo**. Everything else ("exact Gazebo version, TurtleBot configuration, sensor setup, ROS/ROS2 version, software requirements, and submission procedure") is explicitly deferred to a technical rulebook that has not been released as of this writing.

### 1.9 Governance
The organizing committee can change structure at will and its decisions on qualification, outcomes, tie-breaks, and structure are final. Don't build anything that only works if the rules stay exactly as currently worded — build for the *spirit* (fast, robust, reactive pursuit/evasion) so you survive rule tweaks.

---

## 2. Open Questions — Ask the Organizing Committee Now

Every week you wait is a week you might optimize the wrong thing. Send these (WhatsApp group / email) this week:

1. **Round 1 target representation:** Is the standardized Runner a literal ground-truth pose topic (like your own `sim_ground_truth_pose`), a rendered sphere requiring vision, or an actual TurtleBot? This determines whether you need a perception pipeline at all for Round 1.
2. **Round 1 target motion profile:** Is it fixed/deterministic (same every run) or randomized per attempt? If fixed, you can literally hand-tune against it; if randomized, you need general robustness (see §4).
3. **Exact ROS 2 distro and Gazebo version.** Jazzy + Gazebo Sim 8 is your best guess reading "TurtleBot 4 Lite on Ubuntu 24.04," but an official pin avoids last-minute breakage.
4. **Sensor interface for Round 2 (offline).** Will lidar/camera be enabled in the official Gazebo image? If yes, you must resolve the render segfault (§7.1) before the offline round, since your current fallback avoids sensors entirely.
5. **Exact obstacle count/placement for official matches.** "4–6 static obstacles," no coordinates given. Ask if a reference world file will be published, and if placement is randomized per match/team.
6. **Per-robot speed parity.** Are Catcher and Runner guaranteed identical hardware/speed caps (same TurtleBot 4 Lite config), or can teams tune their own robot's max speed independently? This changes whether pure-interception strategy (your current approach) is necessary or whether a speed edge is also viable.
7. **Offline round logistics:** Whose machine runs the sim (yours, a shared venue rig, cloud)? If yours, is there a hardware/OS spec sheet? If shared, what's the setup/verification window before your match?
8. **The "additional standardized tie-breaker."** What is it, concretely? (Sudden-death inning? Shorter arena? Faster clock?)
9. **<8 qualifiers contingency** and exact **bracket seeding rule** (is seeding by Round 1 capture time?).

Log answers in this document's §2 as they arrive — treat it as a living addendum.

---

## 3. Current System Snapshot (as of this audit)

```
turtle_pursuit_ws/
  src/turtle_pursuit/
    turtle_pursuit/
      catcher/      strategy.py (SEARCH→INTERCEPT→CHASE→CAPTURE state machine), node.py
      runner/       strategy.py (baseline/strategic/stationary/standardized), node.py
      planning/     interception.py (constant-velocity forward sample), grid.py (A*, untested-live)
      tracking/     velocity.py (EMA velocity estimator)
      control/      motion.py (accel/vel-limited drive-to-pose controller + reactive scan avoidance)
      evaluation/   node.py (continuous capture-hold detector, JSON/marker output), benchmark.py, match.py
      adapters/     ros_adapter.py (Gazebo↔internal type boundary), sensorless_description.py
    launch/         pursuit.launch.py, sensorless_spawn.launch.py
    worlds/         pursuit_arena.sdf (10×10 room, 4 boxed obstacles at (±2,±2))
    config/         pursuit.yaml (single shared parameter set for all nodes)
    test/           test_algorithms.py (7 tests), test_ros_smoke.py, test_sensorless_description.py
  benchmark_results.csv   (kinematic benchmark: 2 catcher strategies × 7 runner behaviors × 5 seeds)
```

### 3.1 ROS interfaces in play
| Purpose | Topic (per role) | Type |
|---|---|---|
| Ground-truth pose | `/{role}/sim_ground_truth_pose` | `nav_msgs/Odometry`, best-effort QoS |
| Velocity command | `/{role}/diffdrive_controller/cmd_vel` | `geometry_msgs/TwistStamped` **(not plain Twist — easy integration bug)** |
| Lidar (currently unused, disabled) | `/{role}/scan` | `sensor_msgs/LaserScan` |
| Collision/stall | `/{role}/hazard_detection` | `irobot_create_msgs/HazardDetectionVector` |
| Match telemetry | `/match/state`, `/match/catcher_mode`, `/match/runner_mode`, `/match/markers` | `String` (JSON) / `MarkerArray` |

### 3.2 Verified baseline performance
- Headless physics capture vs. a **stationary** Runner: **9.10 s**, min separation 0.37 m, Catcher path 2.63 m. This is your current "does the whole pipeline work" number — not your competitive number.
- Kinematic benchmark (no Gazebo, pure math): predictive Catcher beats baseline against `wall_aware`, `obstacle_weaving`, and `strategic` Runner behaviors (~4.7 s captures vs. baseline's comparable-but-slightly-worse numbers), and both handle `random_walk` fine (~5.2 s).

---

## 4. Critical Finding: The Circular-Motion Blind Spot (P0)

Your own CSV proves this — it is not speculation:

| catcher_strategy | runner_behavior | capture_success | survival_time | minimum_separation |
|---|---|---|---|---|
| baseline | circular | **False** | 60.05 s (of a 60 s test) | 0.599 m |
| predictive | circular | **False** | 60.05 s | 0.597 m |

**Why this happens:** `interception_point()` in `planning/interception.py` extrapolates the Runner's future position with **constant linear velocity** (`runner.x + vx·t`). Against a Runner moving on a circular arc, this is systematically wrong for every horizon step beyond the very short term — the predicted intercept point continuously "leads" the Runner along a tangent line instead of along the arc, so the Catcher chases a moving target that curves out from under the predicted point every cycle. The Catcher closes to ~0.6 m (chase_distance range) and then orbits rather than converging, because inside `chase_distance` the strategy falls back to direct pursuit (`CHASE` mode, target = current Runner position), which is *pure pursuit* and is known to fail to converge on a comparably-fast circularly-moving target (the pursuit curve degenerates into a stable offset orbit rather than a shrinking spiral) unless the Catcher has a real speed or turn-rate advantage.

**Why this matters for the actual competition, not just the benchmark:** a "standardized, reproducible target" is exactly the kind of thing organizers would implement as a simple parametric path — and a circle or oval loop is one of the most natural choices (easy to code, easy to verify, easy to make deterministic). If Round 1's target moves anything like this, your qualification time is either terrible or infinite under the current strategy.

**Fix, in priority order:**
1. **Curvature-aware prediction.** Replace or augment the constant-velocity extrapolation with a constant-turn-rate (CTRV) motion model: estimate not just `vx, vy` but also angular rate `ω` from consecutive heading changes, and forward-simulate the Runner along an arc, not a line. This is a bounded, well-understood extension (a few dozen lines) to `tracking/velocity.py` (add angular-rate EMA) and `planning/interception.py` (arc-based forward sampling instead of straight-line).
2. **Close-range mode fix.** Inside `chase_distance`, don't degrade to "chase current position" — keep using the (now curvature-aware) predicted intercept point all the way to `capture_control_distance`. Pure pursuit-of-current-position is the actual mechanism causing the orbit; removing that fallback (or making the switch distance much smaller) should help even before the CTRV fix lands.
3. **Add `circular` (and ideally `figure_eight`, `oval`) to your CI/benchmark gate.** You already have the harness (`run_benchmarks.sh`) and the `circular` behavior — the gap was never running it as a pass/fail gate. Make "must capture the circular runner in <X s across all seeds" a merge-blocking check before Round 1 submission.
4. **Re-run the full CSV sweep after the fix** and diff against the current numbers to confirm no regression on the scenarios you already win (wall_aware, obstacle_weaving, strategic, random_walk, stationary, straight).

This single fix is plausibly the highest-leverage work item in this entire plan.

---

## 5. Round 1 Playbook — Catcher Qualification

**Goal:** minimize capture time against whatever the standardized target turns out to be, without knowing exactly what it is. Optimize for *robustness across plausible targets*, not for one guess.

### 5.1 Strategy
- Treat the `standardized` Runner mode you already built (fixed 8-waypoint patrol loop) as **one** rehearsal scenario, not the only one. It's a straight-line-segment patroller — good for validating waypoint-chase behavior, useless for validating curved-path robustness.
- Build (or reuse from the benchmark) a **battery of plausible Round 1 targets**: stationary, straight-line, circular/oval loop, waypoint patrol (your existing `standardized`), random walk, and a "sphere/dot" abstraction (a kinematic point with no physical footprint — check your Catcher doesn't rely on any Runner-specific collision/hazard signal that a dot wouldn't emit).
- Your Catcher must pass **all** of these before submission, not just the one you assumed. This is what §4's fix directly buys you.
- Since Round 1 is Catcher-only, do **not** spend Round 1 prep time polishing Runner strategy — that's Round 2 scope. Every hour between now and Sep 12 should go to Catcher robustness, perception-independence (works on ground truth *or* degrades gracefully without lidar), and capture-time optimization.
- Optimize the `CAPTURE` mode's final approach carefully: the tie-break and qualification ranking are literally "who captures fastest," so shaving the last half-second of hold-and-close behavior (e.g., tuning `capture_speed`, `capture_control_distance`, `hold_correction` in `catcher/strategy.py`) has direct scoreboard value even after correctness is solid.

### 5.2 Submission checklist (Round 1: GitHub repo + README + video)
- [ ] **README** clearly states: assumptions made where the rulebook is silent (simulator version, sensor interface), how to reproduce a run end-to-end from a clean machine, and a results table (capture time across your target battery, not just one number).
- [ ] **Video evidence**: record a clean run against your best reproduction of the standardized target, *and* a short compilation showing the Catcher succeeding against 3–4 different target behaviors (circular included) to preempt "did you just overfit to one demo?" doubt from judges.
- [ ] **Repo hygiene**: remove dead code paths, ensure `colcon build && colcon test` passes clean on a fresh clone, pin dependency versions in `package.xml`, and make sure `.gitignore` actually excludes build artifacts (`build/`, `install/`, `log/`).
- [ ] **Timestamp/overlay the video** with the live `/match/state` capture-time readout so judges don't have to trust an edited clip.
- [ ] Submit *before* the deadline with margin — "online" submissions with video evidence are exactly the kind that get bitten by last-minute upload/render failures.

### 5.3 Concrete algorithm checklist before Round 1
- [ ] Circular-motion intercept fix (§4) merged and benchmark-gated.
- [ ] Confirm behavior when the target is a literal point/sphere with no physical hazard emissions (don't let the Catcher stall waiting for a hazard message that will never come from a non-TurtleBot target).
- [ ] Confirm `stale_timeout` (2.0 s) and `SEARCH` fallback behave sanely if the standardized-target pose topic has a different name/rate than your own `sim_ground_truth_pose` assumption — add a config-driven topic remap rather than a hardcoded name, so a last-minute organizer topic name doesn't require a code change.
- [ ] Re-verify the capture-hold logic end-to-end against the *rule's* exact wording (already unit-tested — good — but re-run the Gazebo-level test too, since evaluator timing granularity, not just the pure Python detector, must satisfy "continuous for 1 second").

---

## 6. Round 2 Playbook — 1v1 Knockout

**Goal:** win as Catcher AND win as Runner, since you're scored in both roles every matchup.

### 6.1 Strategy
- **Symmetric threat model.** Assume your opponent has *also* built a predictive/interception Catcher — because the rulebook and public "All you need to know" doc are visible to everyone, all competitive teams are converging on similar architectures (state machines, intercept prediction, evasion scoring). Don't assume you'll face a naive baseline chaser or a naive straight-line fleer.
- **Runner robustness against a good Catcher.** Your `strategic` Runner scores 11 candidate headings using distance-from-catcher, wall/corner clearance, and heading smoothness. Two concrete upgrades to consider before Round 2:
  - Add an explicit **anti-interception term**: if the current fix in §4 gives you a curvature-aware Catcher, assume opponents will too — so your Runner should avoid long constant-curvature arcs itself (since a good opponent Catcher will predict them). Mix in occasional heading discontinuities (already partially done via seeded jitter) but make sure jitter isn't so small it's still locally near-linear/near-circular over the opponent's prediction horizon (~4 s at your own default `prediction_horizon`).
  - Add a **capture-radius escape reflex**: a special high-priority behavior when separation drops below some threshold above `capture_radius` (e.g., 1.0–1.2 m) that prioritizes sharp perpendicular breaks over smooth scoring, since the tie-break rules reward *surviving*, not scoring elegance.
- **Opponent scouting.** Round 1's video submissions may become visible (ask organizers if Round 1 videos are shared/public before Round 2 seeding). If so, review finalist Catcher/Runner behavior for exploitable patterns (fixed intercept horizon, predictable evasion headings, slow replanning) before your Round 2 matchup.
- **Match-day adaptivity.** You get 2 innings, roles swapped, likely with some time between them. Have a lightweight "what did we just learn" checklist your team runs between innings 1 and 2 (e.g., "did the opponent's Catcher favor cutting corners near obstacle b/d? adjust our Runner's clearance weights before inning 2 if config is re-loadable without a rebuild").

### 6.2 Offline-round logistics (this is where teams lose winnable matches on infrastructure, not algorithms)
- [ ] **Portable, reproducible environment.** Package the full stack (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Sim 8 + your workspace) as a Docker image or a bootable/VM snapshot, so venue hardware differences can't break your build on the day. Test this snapshot on a machine you haven't used for development.
- [ ] **Offline capability.** Assume no reliable internet at the venue: no `apt install`, no `pip install`, no cloning at match time. Everything must run from what you bring.
- [ ] **Bring a spare laptop** with the identical verified image, plus a wired network setup if the venue uses a shared network for match orchestration.
- [ ] **Rehearse the exact launch command** (`ros2 launch turtle_pursuit pursuit.launch.py ...`) including the `startup_delay` timing — your README notes ~20 s is the minimum for both namespaced diff-drive controllers to activate; confirm this holds on whatever machine you actually bring, since it's hardware-dependent.
- [ ] **Print (physical paper) copy of the rulebook and your own README/run commands** in case of AV/projector chaos or dead laptop battery mid-event.
- [ ] Know the **exact CLI flags** for both roles (`catcher_strategy:=predictive runner_strategy:=strategic`, `headless:=true rviz:=false`) cold — don't be debugging launch syntax in front of a bracket clock.

---

## 7. Engineering Backlog (prioritized)

### P0 — Must fix before Round 1
| Item | Why | Effort |
|---|---|---|
| Curvature-aware interception (§4) | Confirmed 0% capture rate against circular motion | Medium |
| Benchmark-gate `circular` scenario in CI | Prevents silent regression on the exact bug you just found | Small |
| Config-driven topic names for Round 1 target | Organizer's target topic name is unknown; hardcoding = fragile | Small |
| Verify capture-hold timing at the Gazebo/evaluator level, not just unit test | The rule's 1-second continuous hold is scored by the live evaluator's 20 Hz tick loop, not the pure-Python detector alone | Small |

### P1 — High value, target before Round 2
| Item | Why | Effort |
|---|---|---|
| §7.1 Resolve sensor-rendering segfault | Live obstacle avoidance is currently non-functional; A* has no live feed | Large |
| Runner anti-interception + capture-radius escape reflex (§6.1) | Round 2 opponents will run comparable Catchers | Medium |
| Per-role independent speed/accel tuning support | Currently one shared `pursuit.yaml` for both robots; confirm this is intentional once §2 Q6 is answered | Small |
| Opponent-behavior post-match adaptation hooks | Config reload without rebuild, for between-innings tuning | Medium |

### P2 — Nice to have / polish
| Item | Why |
|---|---|
| Render predicted intercept point / escape target as separate RViz markers | Debugging and demo-video clarity |
| Camera-based fallback target detector | Only needed if §2 Q1 reveals a vision-only target representation |
| Automated video capture/overlay tooling for submissions | Saves manual editing time before deadlines |

### 7.1 On the sensor segfault specifically
Your README already diagnoses this precisely: two robots' full sensor stacks (camera/lidar/cliff/IR) rendering simultaneously crash Gazebo 8's render thread, so the stable launch strips them. Two realistic paths forward, worth spiking early rather than late:
- **CPU-only lidar** (no GPU ray tracing / no camera rendering) is usually the cheapest sensor to re-enable without touching the render pipeline that's segfaulting — investigate whether a `gpu_lidar`→`lidar` (CPU raycast) swap in the TurtleBot 4 Lite description avoids the crash while still feeding your existing (already-implemented, already-tested) `avoid_scan()` and A* planner.
- If GPU rendering is unavoidably the culprit, consider running each robot's sensor-bearing plugins in **separate Gazebo server processes** bridged together, or reduce to a single shared low-rate sensor and infer the other robot's obstacles from ground truth (since you already have privileged sim access) as a pragmatic stopgap for Round 2 — obstacles are *static*, so you can plausibly hardcode/pre-load the known 4–6 obstacle footprints into your occupancy grid rather than requiring live perception at all, unless the rules explicitly forbid using a pre-supplied map matching the arena's official geometry (ask in §2).

---

## 8. Tuning & Benchmarking Methodology

You already have the right tool (`scripts/run_benchmarks.sh`, `evaluation/benchmark.py`). Use it systematically:

1. **Never tune on a single seed.** Your CSV already runs 5 seeds per scenario — keep this minimum, and use more (10+) for parameters you're about to lock in before submission, since seed-to-seed variance is exactly what separates a real improvement from noise.
2. **One parameter at a time.** `pursuit.yaml` has ~20 tunable knobs (`prediction_horizon`, `velocity_alpha`, `chase_distance`, `capture_control_distance`, Runner score weights, etc.). Sweep one axis, hold others fixed, and record the CSV diff — don't change three knobs and one test run and declare victory.
3. **Optimize for the metric that's actually scored.** Round 1 = capture time only. Round 2 Catcher innings = capture time; Round 2 Runner innings = survival + final separation for tie-break. Don't let one shared tuning pass silently trade Runner survivability for Catcher speed (they can conflict, e.g., higher `max_linear` helps Catcher chase but — if genuinely shared/symmetric per §2 Q6 — also helps opponents' Runner instances escape you).
4. **Track a regression table.** Before/after each backlog item, append full-sweep CSV results to a dated snapshot so you can prove (to yourselves, and in the README) that changes were net-positive across the whole scenario battery, not just the one you were fixing.
5. **Reserve the last few days before each round for freeze + validation only** — no new features once you're inside ~72 hours of a deadline; only bug fixes confirmed by the benchmark suite.

---

## 9. Testing & Validation Matrix

| Layer | Tool | Covers | Status |
|---|---|---|---|
| Unit | `pytest` (`test_algorithms.py`) | geometry, velocity EMA, interception math, A* correctness, capture-hold logic, motion limiter NaN-safety, config loading | 7 tests passing — keep growing this with every bug found (start with a regression test for §4) |
| ROS smoke | `test_ros_smoke.py` | Node bring-up sanity | Existing |
| Description | `test_sensorless_description.py` | Sensorless URDF/xacro generation | Existing |
| Kinematic benchmark | `evaluation/benchmark.py` | Cross-strategy, cross-behavior, multi-seed, no Gazebo dependency (fast iteration) | Existing — **extend to gate on `circular`, add `figure_eight`/`oval`** |
| Full Gazebo integration | `pursuit.launch.py` + `EvaluatorNode` | End-to-end physics, real controller timing, collision hazards | Existing — run before every submission, not just in dev |
| Stress / adversarial | *(new)* | Sensor loss mid-match (unplug lidar), stale pose feed, network/topic-rate degradation, opponent Catcher/Runner behaviors you don't control | **Add before Round 2** — simulate a "bad day" (dropped messages, high latency) since offline venue networking is a real variable |
| Clean-machine rebuild | *(process, not code)* | `git clone` → `colcon build` → `colcon test` → launch, on a machine that has never seen this repo | **Run at least once before each submission deadline** |

---

## 10. Risk Register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Round 1 target moves in a pattern your Catcher can't intercept (§4) | High (was already true) | Round 1 elimination | Curvature-aware fix + battery testing (§4, §5.1) |
| Live sensors unusable at Round 2 due to render crash | Medium–High | Obstacle collisions / stalls in offline matches | §7.1 spike now, not the week before |
| Official rulebook changes core numbers late | Medium | Wasted tuning effort | Keep all arena/physics constants in `pursuit.yaml`/`pursuit_arena.sdf`, never hardcoded in strategy logic (mostly already true — audit for stragglers) |
| Venue machine can't run your stack | Medium | No-show / forfeit | Docker/VM snapshot tested on a foreign machine (§6.2) |
| Video submission fails to upload/render before Round 1 deadline | Medium | Missed qualification despite working algorithm | Submit 24–48 h early, compress/export test the exact file format required |
| Opponent has a materially better Runner in Round 2 and your Catcher orbits again under a *different* unseen motion pattern | Medium | Lost knockout match | Broaden benchmark battery beyond `circular` (figure-eight, sudden direction reversals, opponent-style strategic evasion) before Round 2 |
| Shared `pursuit.yaml` assumption about symmetric speed turns out wrong | Low–Medium | Sub-optimal tuning, not a functional failure | Resolve via §2 Q6; current interception-based design is robust either way |
| Team member availability / single point of failure on launch-day operations | Medium | Operational delay during your match slot | Cross-train at least 2 people on the exact launch/verify sequence (§6.2) |

---

## 11. Timeline

| Window | Focus |
|---|---|
| **Now → Sep 5** (P0 sprint) | Curvature-aware interception fix, benchmark-gate `circular`, config-driven topic names, Gazebo-level capture-hold verification |
| **Sep 5 → Sep 10** | Full target battery validation, README + video production, clean-machine rebuild test, early submission |
| **Sep 12–13** | Round 1. Submit early, monitor for organizer clarifications. |
| **Sep 14 → Oct 1** | Digest Round 1 outcome/feedback; if answers to §2 arrived, re-baseline assumptions; start §7.1 sensor-segfault spike |
| **Oct 1 → Oct 20** | Runner hardening (anti-interception, escape reflex), opponent scouting if Round 1 videos are visible, stress/adversarial test suite |
| **Oct 20 → Oct 28** | Docker/VM portability validation on foreign hardware, full dress rehearsal of both roles end-to-end, freeze new features |
| **Oct 28 → Nov 1** | Final logistics (spare laptop, printed docs, launch-sequence rehearsal), Round 2 |

---

## 12. Appendix

### 12.1 Quick command reference
```bash
# Build & test
cd turtle_pursuit_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+ && colcon test-result --verbose

# Run a full match
ros2 launch turtle_pursuit pursuit.launch.py \
  headless:=true rviz:=false world:=pursuit_arena seed:=1 \
  match_duration:=180.0 catcher_strategy:=predictive runner_strategy:=strategic

# Kinematic benchmark sweep
bash scripts/run_benchmarks.sh 10 180
```

### 12.2 Key config knobs (`config/pursuit.yaml`)
`max_linear`, `linear_accel`/`angular_accel`, `prediction_horizon`/`prediction_step`, `velocity_alpha`, `chase_distance`, `capture_control_distance`, `capture_radius`/`capture_hold`, `boundary_margin`, Runner weights (`distance_weight`, `clearance_weight`, `open_weight`, `smooth_weight`), `stale_timeout`, `planning_resolution`, `obstacle_inflation_radius`.

### 12.3 Glossary
- **CTRV:** constant turn-rate and velocity motion model — the standard fix for tracking curved-path targets, versus the current constant-velocity (straight-line) model.
- **Pure pursuit:** a controller that always steers toward the target's *current* position rather than a predicted future position; converges poorly against comparably-fast curved-path targets.
- **EMA:** exponential moving average, used here to smooth noisy velocity estimates (`velocity_alpha`).

---

*This document should be treated as living — update §2 as organizer answers arrive, and append new rows to the benchmark regression table (§8) as fixes land.*
