# Physical gripper calibration and stopping — 2026-10-23

The operator confirmed that the real adaptive gripper was closed, then confirmed
an empty gripper, a firmly supported arm and clear space for a small opening and
stop test. No joint-position commands were sent to the six arm actuators.

Connection: `/dev/ttyACM0`, 115200 baud, `ros2-humble`, ROS domain 42.
Device reports system version 7.2 and basic version 2.4.

## Findings

- The nominal conversion mapped closed readings of 15 to -0.6065 rad, outside
  the compatible model limits. After a stop, the closed reading settled at 12.
  Both hardware YAML files now map 12 to -0.5 rad. The open endpoint 100 to
  0.15 rad remains nominal. Measurements are not clamped; model limits remain
  unchanged. This is not a calibration of jaw width or contact forces.
- SDK torque stopping acknowledged with 1, but during actual movement the
  gripper continued changing position and did not report stationary within
  the one-second controller deadline. A preceding stationary test alone was
  insufficient to expose this.
- A four-argument position command including `is_torque=0` did not produce a
  measured opening in this test. It is not used in the fix.
- A three-argument position command to the current measured device position
  stopped the gripper successfully. The movement test began at 14, requested
  22 at speed 5, observed movement to 17, then requested a hold at 17 at speed
  10. It briefly reached 20, returned to 17 and reported stationary within
  the one-second deadline; five additional samples remained 17 and stationary.
  The six arm angles were unchanged across this test. Raw evidence:
  [2026-10-23-physical-gripper-stop.json](2026-10-23-physical-gripper-stop.json).

## Implementation and verification

`gripper_stop_strategy=hold_position` selects the tested read-and-hold command.
It shares the sole USB transport lock, remains usable after session revocation,
requires a valid device reading, and preserves the read-only write gate. Failed
stopping still latches the motion fault. No automatic torque-release fallback.
Existing deployments without the YAML retain the original `torque` default.

Eight new regression cases failed before implementation. After implementation,
the functional Python suite passed: **132 passed, 3 inherited linters deselected**.
An existing xacro warning remains. Log inside the container:
`/tmp/mycobot-hold-stop-tests.log`.

The current physical driver was restarted with `allow_motion=true`, calibrated
closed endpoint 12, `hold_position` and `gripper_stop_confirmed=true`. Startup
sent no motor commands. The public `/mycobot_control/select_physical` service
accepted selection (`Destino: brazo físico`) and deselection (`Destino: Gazebo`).
Final physical status: connected, unselected, live off, no fault. RViz and Gazebo
remain running. Only the operator's later selection and execution should command
the arm. Full physical arm trajectories/live following and loaded stopping have
not been validated by this test.

For a future session on this verified device, use:

```bash
ros2 launch mycobot_280_hardware control.launch.py \
  adaptive_gripper:=true allow_motion:=true gripper_stop_confirmed:=true
```

Do not start this alongside the current session; it would duplicate USB ownership
and ROS nodes. The session launches into Gazebo and requires explicit physical
selection in RViz.

After validation, the temporary standalone driver and reopened GUI processes
were stopped. A single unified launch now owns the entire session, including
the USB driver. Its log is `/tmp/mycobot-unified-physical-session.log`. This
ensures stopping that launch also shuts down the hardware driver.
