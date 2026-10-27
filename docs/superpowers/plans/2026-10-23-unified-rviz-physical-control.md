# Unified RViz Physical Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans. The user explicitly requires inline work without agents or commits and has approved implementation.

**Goal:** One RViz selects Gazebo or the connected arm for live following and Plan & Execute.

**Architecture:** An integrated launch isolates Gazebo under `/simulation` and USB under `/physical`. A coordinator publishes only selected measured states, routes trajectory actions, guards MoveIt plans by selection generation, and adapts the existing live controller-manager interface to the selected destination. A revocable hardware session lives in the existing single-owner USB driver, with an independent monotonic watchdog. Existing standalone launches remain compatible.

**Tech Stack:** ROS 2 Humble, Python/rclpy, C++17/Qt RViz, MoveIt Servo, Gazebo Classic, pymycobot.

**Spec:** docs/superpowers/specs/2026-10-23-unified-rviz-physical-control-design.md (approved “Dale”).

## Global Constraints

- Same RViz, exclusive destination; no automatic motion or rearm on USB connection.
- `allow_motion=false` and `gripper_stop_confirmed=false` defaults remain.
- No physical test motion; use fake devices and isolated Gazebo integration.
- Real-time planning/state presentation; monotonic session watchdogs; simulation progress checked independently.
- Gripper command range [-0.5, 0.15] rad; physical live requested joint speed <=0.1 rad/s, serial <=10Hz.
- No agents, commits, unrelated cleanup, or changes to the running domain 42 until isolated validation passes.

## Review Focus

- Stale plan after returning to a previously selected destination must remain rejected.
- Delayed session commands and old heartbeat must not revive a revoked session.
- Stopping without a valid nonmoving reading must latch a fault.
- Paused simulation or stale serial samples must not become fresh by republishing.
- Missing physical arm/gripper must leave simulation available without exposing active hardware.

### Task 1: Hardware session and protocol

**Files:** new `mycobot_280/mycobot_280_control_interfaces/{msg/HardwareCommand.msg,msg/HardwareStatus.msg,srv/SetHardwareMode.srv,CMakeLists.txt,package.xml}`; new hardware `hardware_session.py`, `hardware_live.py`, test `test_hardware_session.py`, `test_hardware_live.py`; modify driver and gripper executor.

**Interfaces:** SetHardwareMode(selected, live, session_id) returns success, unique session_id, message. HardwareCommand carries wall stamp, session ID, sequence and complete principal positions. HardwareStatus carries connected/selected/live/fault/session/message.

- [x] Write pure-policy failing tests for stale/invalid readings, session lifetime, sequence/age rejection, heartbeat timeout, fault latch, no auto-rearm and rate/step limits; run pytest and observe RED.
- [x] Implement HardwareSession policy with monotonic deadlines, strict generation and immutable command validation; run GREEN.
- [x] Write fake-device failing tests for no startup commands, session/action exclusion, refresh-mode verification, actual stop verification and gripper-specific stop.
- [x] Integrate HardwareLiveController with RobotController single USB owner, wall timer and direct-action guards; keep legacy standalone behavior unless `require_session` is true. Run hardware tests.

### Task 2: Coordinator and stale-plan guard

**Files:** hardware `control_coordinator.py`, `selection_policy.py`, tests `test_selection_policy.py`, `test_control_coordinator.py`; setup/package dependencies.

**Interfaces:** public `/mycobot_control/select_physical` SetBool, `/mycobot_control/physical` Bool and `/mycobot_control/status` String. Public canonical FJT, MoveGroup, ExecuteTrajectory actions route to simulation/physical FJT and `/internal` MoveIt actions. Canonical controller-manager list/switch are a live-transport adapter, never a second USB owner.

- [x] Write failing tests for immutable destination during jobs, session revocation, measured-state selection, simulation progress, per-generation trajectory fingerprints and no stale-plan reuse. Run RED.
- [x] Implement policy and coordinator with bounded waits, feedback/cancel/result propagation, stop on backend loss and physical heartbeat conditional on RViz heartbeat.
- [x] Test with fake ROS action/service backends: simulation routing, selection rejection, physical execution permissions, watchdog teardown and old-plan rejection. Run GREEN.

### Task 3: RViz destination button

**Files:** live panel header/source/test; reuse selected live interface without changing legacy mode.

**Interfaces:** button requests selection only; selected Bool is authoritative, status visible even while follow is off. Hide/disable selector if coordinator absent, retain previous simulation behavior.

- [x] Add failing Qt/ROS tests for confirmed physical selection, rejected/pending changes and disabled selector while follow active.
- [x] Add button/status with asynchronous requests; preserve existing panel tests and heartbeat.
- [x] Build C++ and run both CTests offscreen.

### Task 4: Integrated launch and namespace isolation

**Files:** Gazebo launch/xacro optional namespace support; new hardware launch `control.launch.py`; MoveIt launch support internal action remaps and real-time integrated mode; hardware dependencies/setup.

**Interfaces:** `ros2 launch mycobot_280_hardware control.launch.py adaptive_gripper:=true allow_motion:=false port:=/dev/ttyACM0` starts one RViz, isolated Gazebo, one USB driver and coordinator; use_rviz/gui options support headless tests.

- [x] Test xacro/launch generation, controller-manager namespace, only selected state source and no TF duplicate authority.
- [x] Implement integrated launch while preserving default existing launch behavior.
- [x] Build all affected packages; run functional pytest and CTest.

### Task 5: End-to-end validation and documentation

**Files:** integration tests under hardware/test; README and hardware README; docs/testing/2026-10-23-unified-control-validation.md.

- [x] Isolated Gazebo + fake USB test both modes, live arm/gripper, Plan & Execute, selection conflicts, paused Gazebo, watchdog loss, stale plan, serial fault/reconnect without rearm.
- [x] Inspect panel screenshot in one RViz; preserve the current user session until validation permits replacement.
- [x] Review new code inline against spec, fix concrete findings with regression tests; run verification and diff check.
- [x] Document exact Docker commands, USB exposure and conservative defaults; clearly distinguish software tests from unperformed physical validation.
