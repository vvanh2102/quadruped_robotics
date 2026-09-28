# Roadmap — vanh quadruped

Architecture reference: Nishio project (`~/Documents/Nishio/ros/`). Some gait/IK algorithms will be referenced from `Orion-Quadruped-master` (`~/Documents/Orion-Quadruped-master/`). Update this file directly as work progresses — tick `[x]` when done, note date/decision if direction changes.

Convention:
- `[ ]` not started — `[~]` in progress — `[x]` done

---

## 1. vanh_ros_simulator
Role: main brain — URDF/ros2_control, MoveIt, trajectory execution. Equivalent to `nishio_ros2_moveit2`, used for planning/preview only, does not directly drive real hardware. Package name on disk: `vanh_ros_simulator` (already exists as an empty skeleton in `src/`).

**First concrete step (locked 2026-09-25):** subscribe `/robot_info` (published by `vanh_ros_control`), map its 12 feedback angles to the correct URDF joint names, republish as `sensor_msgs/JointState` on `/joint_states` for `robot_state_publisher` → RViz. When this node runs, `joint_state_publisher`/`joint_state_publisher_gui` in `view_model.launch.py` must be disabled — only one publisher on `/joint_states` at a time.

- [~] `/robot_info` → `/joint_states` bridge node (see design below)
- [ ] Confirm ROS 2 distro (Humble/Jazzy) before writing code
- [ ] Add `<ros2_control>` block to xacro (12 joints, `command_interface position`, `state_interface position`)
- [ ] Use `mock_components/GenericSystem` as the hardware plugin for now (not real hardware)
- [ ] Write `controllers.yaml`: `joint_trajectory_controller` + `joint_state_broadcaster`
- [ ] Launch `controller_manager`, verify with `ros2 control list_controllers` / `list_hardware_interfaces`
- [ ] Test sending a trajectory manually (action client or `ros2 topic pub`) before wiring up MoveIt
- [ ] Integrate MoveIt: SRDF, kinematics.yaml, planning pipeline (reference Nishio's `config_v3/`)
- [ ] Write a dedicated trajectory-execute node if MoveIt's default execution pipeline isn't flexible enough for high-frequency gait (reference `nishio_trajectory_execute_node.cpp`)
- [ ] (Optional) Gazebo for real physics when balance/foot-contact needs checking

## 2. vanh_ros_control
Role: hardware/sim communication — does NOT use ros2_control. Plain rclpy/rclcpp node, picks one of two interfaces sharing the same API via a `simulation` param. Equivalent to `nishio_ros_control`.

**`/robot_info` schema (locked 2026-09-25):** dedicated message, feedback only (never carries commanded angles — commands are a separate topic/service, added later alongside PS5 manual/auto mode). Fields: `header`/timestamp + 12 joint angles in **rad**. `vanh_ros_control` is the only publisher, in both Sim and STM32 mode — RViz must see the same real feedback path regardless of mode. Bootstrap/test plan: manually publish a sample `/robot_info` message before PS5 or STM32 exist, to validate the bridge in `vanh_ros_simulator` end to end.

- [~] Define `/robot_info` message (header + 12x float64 positions, rad, feedback-only) — equivalent to Nishio's `RobotInformation`
- [ ] Define command message/service for manual/auto mode later — equivalent to Nishio's `AutoControl`/`ManualControl` (deferred, not blocking `/joint_states`)
- [ ] `Sim_Interface`: numeric joint simulation (angle limits, interpolated speed) — no physics engine needed, runnable right away
- [ ] Main node: publish `/robot_info` on a timer, pick interface (Sim/STM32) via `simulation` param
- [ ] `Uart_Interface`: write the skeleton first (same method signature as Sim_Interface), no need to run against real hardware yet
- [ ] Define the UART protocol with STM32 (in parallel with the step below — framing/checksum must be agreed before coding both sides)

## 3. vanh_stm32
Role: firmware controlling the 12 joints. Only starts once real hardware exists.

- [ ] Leave as an empty skeleton for now, not implemented
- [ ] Once hardware exists: implement the UART protocol agreed on in `vanh_ros_control`
- [ ] Read encoder/feedback joint angles, return them at a fixed cycle
- [ ] Receive angle setpoints, drive motors (PID or existing driver depending on motor type)

## 4. vanh_working_planner
Role: process point cloud data from the 2D lidar, feeding foot placement / obstacle avoidance. Equivalent to `nishio_working_planner` but simpler since input is 2D. Orion's Jetson workspace has a similar split worth checking (`Software/Jetson/workspace/isaac_ros-dev/src/orion_lidar`, `orion_navigation`, `orion_msgs`).

- [ ] Nail down the concrete goal: obstacle avoidance while walking, terrain-aware foot placement via local map, or both
- [ ] Loader for 2D lidar data (`sensor_msgs/LaserScan` or `PointCloud2` depending on the lidar driver)
- [ ] Basic processing logic (noise filtering, obstacle clustering)
- [ ] Output: decide the data format the gait planner consumes (must be agreed before coding both sides)

## 5. vanh_tf
Role: adapter — looks up standard tf2 transforms and republishes them in the format/units other nodes need (not a replacement for tf2). Equivalent to `nishio_tf`.

**Finding 2026-09-28 — the Nishio precedent for this package is dead code.** `nishio_tf` publishes `moveit_target` / `moveit_nozzle`, but the only subscribers are in `nishio_trajectory_execute.cpp`, which `CMakeLists.txt` does not build (it compiles `nishio_trajectory_execute2.cpp` + `nishio_trajectory_execute_node2.cpp` instead — lines 59 and 69). The live sources reference neither topic. So `nishio_tf` is a legacy branch, not part of the running system — treat it as a weak justification for building `vanh_tf`.

General caution when reading Nishio: prefer the `*2.cpp` variants; the non-`2` files are stale and can mislead.

- [ ] Defer until it's clear which node needs a foot pose without wanting to do its own tf2 lookup (gait planner? control node?)
- [ ] If needed: determine source/target frames (base_link → each foot link), output units, publish rate

---

## Gait / IK (referencing Orion-Quadruped-master)

Orion has two IK/gait implementations worth comparing before picking one:
- `Software/STM32Firmware/Orion-Controls/src/LegIK.cpp` + `main.cpp` — per-leg analytic IK (hip/femur/tibia, law-of-cosines solve), gait functions (`stepGait`, `sineStepGait`, `unisonGait`) run directly on the STM32, no ROS involved.
- `Software/kinematics_sim/matplotlib_simple_sim/kinematics.py` — matrix-based IK per leg with body rotation + center-of-rotation offset support (adapted from an external IK reference), meant for simulation/prototyping in Python, not firmware.

**DECIDED 2026-09-28 — gait + IK live in ROS (`vanh_ros_simulator`), not on the STM32.**

Why: if IK ran on the STM32 (Orion's approach), the same math would have to exist twice — C++ on firmware to drive the servos, and Python in `fake_interface` to simulate the resulting 12 angles for RViz. Those two copies drift apart, and then the sim no longer predicts real behavior. With IK in ROS it is written once; its output is 12 joint angles, and both `fake_interface` and `robot_interface` just consume those angles without knowing anything about gait/IK.

Note: the STM32 still converts angle (rad) → servo PWM, including per-joint center offsets and left/right direction inversion (Orion keeps this in `LegIK.cpp` alongside the IK, e.g. `SERVO_CENTER_HIP`, `IS_LEFT_LEG`). That part is hardware-specific and belongs on firmware either way — it is not IK.

Consequence — two command messages at two different levels:
```
PS5 → body velocity (lin_x, lin_y, ang_z, roll, pitch, z_offset)   <- body-level msg
   → gait planner + IK  (in vanh_ros_simulator)
   → 12 target joint angles                                        <- joint-level msg
   → vanh_ros_control (fake_interface / robot_interface)
   → /robot_info (12 feedback angles)
   → /joint_states → RViz
```
- [ ] Joint-level command message (12 target angles) — needed next, for `vanh_ros_control`
- [ ] Body-level command message (Orion `OrionMotionCmd` style) — sits between PS5 and the gait planner, belongs to `vanh_ros_simulator`, deferred
- [ ] Port/adapt the IK math (reference files above) into `vanh_ros_simulator`

Reference notes on how the two projects handle manual/auto (read 2026-09-28):
- **Nishio manual** = per-joint jog by direction. `ManualControl.msg`: `int8[] joints` + `int8[] actions` with `STOP=0 / FORWARD=1 / BACKWARD=-1`. Sim integrates `joint_state[i] += dt * SPEED[i] * action[i]`, then clamps to limits. Suits a hydraulic arm (one lever per cylinder), not a walking gait.
- **Nishio auto** = task-space target. `AutoControl.msg`: `int8[] joints` + `float32[] positions` (`NOZZLE_X/Y/Z` Cartesian, `BOOM_LENGTH`...) + velocity fields; the PLC solves IK. Sim interpolates with a speed cap: `joint_state[i] += min(dt*SPEED[i], abs(target-cur)) * sign(target-cur)`.
- **Nishio mode switching** = `ChangeMode` service sets `self.mode`; the `@runInThread` loop `__getAllStatus()` branches per mode (AUTO interpolates to target, MANUAL integrates direction, else hold). Two separate topics: `manual`, `auto_command`.
- **Orion manual** = `joy_node` → `joystick_parser_node` (PS5 button map) → `/joy_motion_cmd` → `cmd_mux_node` → `/orion_motion_cmd` → `stm32_bridge_node` → UART → STM32 generates gait + IK. `OrionMotionCmd.msg` is body-level only (`cmd_type`, `lin_x/lin_y/ang_z`, `roll/pitch/yaw/z_offset`, offsets, pivot) — no joint angles at all.
- **Orion auto** = not implemented. `cmd_mux_node` has `nav_motion_cmd` commented out with `TODO(orion): Create a unified state machine to handle nav and joystick`; `orion_navigation` only includes nav2 bringup.
- **Orion arbitration** = no mode service; `cmd_mux_node` caches the latest command and republishes at 50 Hz, with no deadman switch ("No longer a deadman switch" in their own comment) — if the joystick stops publishing, the last command keeps repeating. Worth avoiding.
- **Orion UART frame** = header `0x55 0xAA` + `struct.pack('<B11f', cmd_type, lin_x, ...)` + 1 checksum byte; telemetry back is 24 little-endian floats.

Design direction chosen for vanh: Nishio's explicit mode state machine (`ChangeMode` service + `ROBOT_MODE`, already written) combined with Orion-style body-level command content for the PS5/auto layer — Orion is stuck on that TODO precisely because it lacks the mode state machine.

---

## Suggested order

1. `vanh_ros_control` with `Sim_Interface` — runnable immediately, no hardware dependency
2. `vanh_ros_simulator` with `mock_components/GenericSystem` — verify the 12 joints via MoveIt/RViz
3. Basic gait/IK, referencing Orion-Quadruped-master
4. `vanh_working_planner` once real or simulated lidar data is available
5. `vanh_tf` once a concrete need arises
6. `vanh_stm32` + `Uart_Interface` once hardware exists
![alt text](image.png)