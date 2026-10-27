# mycobot_280_hardware

Hardware integration package for the physical **Elephant Robotics myCobot 280 M5** on ROS 2 Humble.

Provides:
1. **`robot_monitor`** (`robot_monitor.launch.py`): strictly read-only `/joint_states` monitor.
2. **`robot_control`** (`robot_control.launch.py` & `moveit_real.launch.py`): single-owner USB serial node that publishes `/joint_states` and serves `/mycobot280m5_group_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) for MoveIt 2 **Plan & Execute**.

## Safety Guarantees

- **Motion disabled by default**: `allow_motion` defaults to `false` in `config/robot_control.yaml`, `robot_control.launch.py`, and `moveit_real.launch.py`. When `allow_motion:=false`, MoveIt 2 can plan from the real robot state, but any execution goal is rejected.
- **No startup motion or servo release**: neither node ever calls `release_all_servos`, `release_servo`, `power_on`, or sends angles on startup.
- **Single serial owner**: a single `ControlledArm` instance guarded by a reentrant lock owns `/dev/ttyACM0`, avoiding serial collisions between `/joint_states` polling and trajectory execution.
- **Pre-execution validation**: checks joint names, finiteness, URDF position limits, velocity limits, monotonic timestamps, and start-state deviation against a fresh measured pose.
- **Smooth point-to-point trajectory coalescing & closed-loop supervision**: coalesces collinear/micro-step MoveIt 2 TOTG subdivisions to prevent servo stop-and-go and firmware queue saturation while preserving corner/reversal via-points, dispatches non-blocking serial commands (`_async=True`), publishes time-interpolated `FollowJointTrajectory.Feedback`, verifies final arrival within `goal_tolerance_rad`, and runs a verified `stop()` procedure on cancellation, timeout, or serial failure.
- **No simulation or duplicate publishers**: uses `use_sim_time=false` and starts no Gazebo, `ros2_control`, or `joint_state_publisher(_gui)`.

## Usage (Physical Robot — Operator Supervised)

Before launching with `allow_motion:=true`, confirm:
1. Workspace around the manipulator is clear.
2. Emergency stop / power switch is within reach.
3. Joint offsets/signs and conservative speed limits (`max_speed:=30`) are reviewed.

### 1. Read-Only Monitor (`robot_monitor.launch.py`)

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch mycobot_280_hardware robot_monitor.launch.py port:=/dev/ttyACM0
```

### 2. MoveIt 2 Plan-Only Mode (Safe Dry-Run Default)

Starts `robot_control` (`allow_motion:=false`), `robot_state_publisher`, `move_group`, and `rviz2`:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch mycobot_280_hardware moveit_real.launch.py port:=/dev/ttyACM0
```

### 3. MoveIt 2 Plan & Execute on Physical Robot (Explicit Opt-In)

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch mycobot_280_hardware moveit_real.launch.py \
  port:=/dev/ttyACM0 allow_motion:=true max_speed:=25
```

Or run the hardware controller and MoveIt 2 in separate terminals:

```bash
# Terminal 1: Hardware controller + robot_state_publisher
ros2 launch mycobot_280_hardware robot_control.launch.py \
  port:=/dev/ttyACM0 allow_motion:=true max_speed:=25

# Terminal 2: MoveIt 2 + RViz connected to the running controller
ros2 launch mycobot_280_hardware moveit_real.launch.py launch_driver:=false
```

## Automated Tests

```bash
colcon build --symlink-install --packages-select mycobot_280_hardware
colcon test --packages-select mycobot_280_hardware
colcon test-result --verbose
```

Mock/unit tests never count as physical hardware validation.

## Adaptive gripper (myCobot 280 M5 / Atom)

The optional model and hardware interface are enabled with
`adaptive_gripper:=true`. The original six-joint behavior remains the default.
Use `/root/ros2_ws/install/setup.bash` for the workspace inside the Docker
container used in this project, and domain 43 for hardware (42 for simulation).
The container needs the actual serial port passed with `--device`.

Start with the read-only monitor:

```bash
ros2 launch mycobot_280_hardware robot_monitor.launch.py \
  port:=/dev/ttyACM0 adaptive_gripper:=true
```

Stop that monitor before starting the controller: both use the USB port.
Plan from the measured arm/gripper state with motion blocked:

```bash
ros2 launch mycobot_280_hardware moveit_real.launch.py \
  port:=/dev/ttyACM0 adaptive_gripper:=true allow_motion:=false
```

`arm_group` controls the six arm joints; `gripper_group` controls
`gripper_controller` using
`/mycobot280m5_gripper_controller/follow_joint_trajectory`. Both action servers
share one USB owner and reject overlapping arm/gripper executions.

### Calibration and stop support before physical execution

The nominal mapping is `-0.74 rad -> device value 0` and
`0.15 rad -> device value 100`. Compatible mimic limits restrict commanded
angles to `[-0.5, 0.15]`, corresponding approximately to device values 27–100.
The named `closed` pose is the model-compatible closure, not a calibrated fully
closed physical jaw. Device values are not jaw width in millimeters.

Verify the mapping, direction and physical limits against read-only measurements
before motion. Calibration parameters (`gripper_closed_rad`, `gripper_open_rad`,
`gripper_closed_value`, `gripper_open_value`) live in `robot_control.yaml` and
`robot_monitor.yaml`; keep both consistent. The supplied YAML files now contain
this workstation's observed closed endpoint: device value **12** maps to
**-0.5 rad**. The open endpoint **100 -> 0.15 rad remains nominal**, and must be
checked on other devices. Initial closed readings were 14–15 before settling.
The conversion class defaults above describe the original, uncalibrated mapping.
Measurements are never clamped to
pretend that an out-of-range physical state is a valid model state. If the current
position is outside the model range, the gripper action rejects execution.

The installed pymycobot SDK implements `gripper_stop()` through a torque-mode
command. On this device (system 7.2 / basic 2.4), a supervised movement test did
not confirm stopping within the controller's one-second deadline. The supplied
controller YAML therefore selects **`gripper_stop_strategy: hold_position`**:
read the actual device position and send that position as a new gripper target
at speed 10. This bypasses revoked session authorization only for stopping,
requires a valid measured value, and still requires acknowledgement and a
non-moving state. It never falls back to torque release on failure.

A supervised empty-gripper test verified this strategy with a small opening;
there was a short braking displacement. This is not an emergency-stop guarantee
or a validation under load. The Python node retains `torque` as its fallback
default for existing deployments without this YAML. Method existence alone is
insufficient. `gripper_stop_confirmed` therefore defaults to
false; gripper goals remain rejected until stop behavior has been verified on the
actual device. No automatic initialization, zeroing or torque calibration runs
at startup. Arm `stop()` is not used as a substitute for gripper stopping.

After those physical checks, an operator can explicitly enable execution:

```bash
ros2 launch mycobot_280_hardware moveit_real.launch.py \
  port:=/dev/ttyACM0 adaptive_gripper:=true \
  allow_motion:=true max_speed:=15 gripper_stop_confirmed:=true
```

Set MoveIt velocity and acceleration scaling to 0.1 for the first small supervised
movements. Keep the power switch accessible. The gripper firmware speed defaults
to 15 (configurable in `robot_control.yaml`, limited to 1–30). Cancellation/errors
use the gripper-specific stop and require acknowledgement and a measured
non-moving state. Failure to verify stopping is reported as an aborted goal.

The hardware controller publishes measured principal gripper position alongside
six measured arm positions on `/joint_states`. Invalid readings are omitted;
execution never reports success from the commanded position alone. The serial
firmware executes position goals rather than a precisely synchronized servo
trajectory, so simulated and physical timing need not match exactly.

Software/fake-device and Gazebo tests do not validate the physical gripper.
No hardware was connected during development of this integration.

If stopping cannot be verified, a latched motion fault blocks both arm and gripper
objectives. Before restarting the controller, physically verify that the robot
has stopped and resolve the communication/firmware fault. A failed stop is never
cleared automatically by accepting a new trajectory. Gripper execution deadlines
use a monotonic clock; ROS timestamps remain available for state and feedback.

## Same RViz: simulation / physical destination selector

`control.launch.py` starts Gazebo, the existing live-follow panel, one MoveIt
instance, the selection coordinator and the session-protected USB driver.
It starts in simulation. The **Manipulador físico** button selects the real arm
for both Plan/Execute and live following; it does not mirror commands to Gazebo.
RViz displays the measured state of the selected destination. Switching requires
both execution and live following to be stopped and invalidates cached plans.

For preview and approval, stop following, select the destination, choose the
current start state, adjust the MotionPlanning goal, then **Plan**, inspect the
preview and **Execute**. Switching destination requires a new plan. A planning
request during following returns a controlled failure and sends no trajectory.

**Physical Cartesian accuracy remains unresolved (26 October 2026).** Bounded
real tests requested +3 mm Z in live mode and +5 mm through separate Plan/Execute;
encoder-based FK instead showed downward endpoint changes. The existing 0.04 rad
joint tolerance can accept a trajectory without meeting such a small Cartesian
goal. Do not treat plan approval or the software direction correction as proof
of accurate physical tracking. The real arm was left stopped with Gazebo selected.
See [reproduction and measured limitations](../../docs/testing/2026-10-26-plan-cartesian-physical.md).
The physical follower now stops on measured travel away from the Cartesian goal
or lack of Cartesian progress. This is a stop guard, not a precision correction.
Explicitly returning to Gazebo after physical session revocation no longer waits
for a disconnected USB driver. Simulation uses its normal planning limits;
physical requests retain the previous conservative limits.
See [destination recovery tests](../../docs/testing/2026-10-26-destination-recovery.md).


```bash
# Inside the container, after sourcing ROS and /root/ros2_ws/install/setup.bash:
ros2 launch mycobot_280_hardware control.launch.py adaptive_gripper:=true
```

The default `allow_motion:=false` keeps physical selection blocked. After checking
joint calibration, workspace, and actual firmware arm/gripper stopping:

```bash
ros2 launch mycobot_280_hardware control.launch.py \
  adaptive_gripper:=true port:=/dev/ttyACM0 \
  allow_motion:=true gripper_stop_confirmed:=true
```

`gripper_stop_confirmed:=true` is an operator assertion, not a firmware test the
launch performs. With the gripper model enabled, the integrated physical session
is rejected until that assertion is provided. Live activation additionally sets
and reads back firmware refresh mode 1 (latest command); unavailable/unknown mode
rejects activation. That mode remains enabled after live following, so the device
does not resume a queued stream. The original standalone launches retain their
existing defaults and execution interface.

### Docker USB access on this computer

The original container had no USB devices passed through. At the initial time
of development neither `/dev/ttyACM*` nor `/dev/ttyUSB*` was present on the host.
`docker exec` cannot add `--device` to an existing container. When the robot is
connected, verify the actual port on Ubuntu before starting a hardware container.

The following commands run on **Ubuntu, outside the container**. They preserve
the current container and reuse its installed ROS dependencies and workspace:

```bash
# Stop the ROS launch in the existing container first (Ctrl+C in its terminal).
docker commit ros2-humble mycobot-humble:local

docker run -it --name mycobot-hardware --network host \
  --device=/dev/ttyACM0:/dev/ttyACM0 \
  -e ROS_DOMAIN_ID=42 -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/mycobot-Xauthority \
  -v "$XAUTHORITY":/tmp/mycobot-Xauthority:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/skuarex/mycobot_ros2_packages:/root/mycobot_ros2_packages \
  -v /home/skuarex/ros2_ws:/root/ros2_ws \
  mycobot-humble:local bash
```

Use an available container name if `mycobot-hardware` already exists; do not remove
an existing container to resolve a naming conflict. Inside the new container:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ls -l /dev/ttyACM0
# Start with physical motion disabled:
ros2 launch mycobot_280_hardware control.launch.py adaptive_gripper:=true
```

The two containers share the workspace. Run only one integrated session in domain
42; use another ROS domain and Gazebo master port for a separate simulation.
After USB reconnect, verify the port is still accessible; Docker device mappings
may need recreation if the kernel assigned a different device. Reconnection never
restores a physical motion session automatically.

### Session and failure behavior

The integrated driver sets `require_session=true`. Both trajectory actions reject
execution without a fresh selected physical session, while live following blocks
trajectory actions. Only this driver owns USB. Its independent monotonic watchdog
revokes the session on missing heartbeat (0.75 s), complete measured state (0.8 s),
or live commands (0.35 s). The coordinator forwards heartbeats only while RViz is
alive. Commands include a session generation, increasing sequence and recent
wall timestamp; stale-session packets never renew permission.

The USB dispatch gate checks permission again just before writing, including when
an already accepted trajectory loses its session. Live output uses only the latest
consigna at up to 5 Hz, requested joint increments <=0.1 rad/s, measured lead
<=0.03 rad for advancing commands, and a no-progress timeout. Unchanged joint
commands already within the verified goal tolerance retain their fixed setpoint
instead of chasing encoder residuals. The firmware percentage speed is capped at
10 for arm and gripper. These are software command limits; firmware timing and
actual measured speed require physical validation.

Live packets additionally carry independent arm/gripper intent flags. Activation
alone renews communication without position writes. A marker drag enables the
arm; changing the slider enables the gripper. See the [activation and axis
validation](../../docs/testing/2026-10-26-explicit-live-intent.md), including the
RViz shutdown workaround and NVIDIA-enabled container used on this workstation.

The coordinator rejects unknown or previous-selection MoveIt plans, including
when switching away and back to the same destination. It hashes trajectory
content rather than ROS wire padding. Cancellation/results/feedback are forwarded
to the selected backend. A failed backend does not silently route a pending goal
somewhere else.

When stopping, the session driver requires a valid nonmoving arm reading as well
as valid angles; a successful serial write alone is insufficient. The gripper
uses its separately verified stop. Failure latches a motion fault. USB loss can
prevent stop delivery entirely: physically check stopping before restarting the
driver. The hardware destination remains selected with a fault indication; it
never quietly turns into simulation. With a confirmed clean stop, explicitly
select the destination again before activating following. If the live backend
reports recovery, the first request restores trajectory mode before following
can be enabled again.

Development verification uses `test/fake_hardware.py` in an isolated ROS domain.
That emulator never opens USB. Its tests are not validation of real firmware,
contact forces, stop distance, calibration or physical safety.

### Contenedor ya preparado en este equipo

El contenedor activo **`ros2-humble` ya tiene `/dev/ttyACM0` expuesto**, los mismos
volúmenes y las dependencias anteriores. Se conservó el contenedor original con
el nombre `ros2-humble-sin-usb-20261023`; no ejecutes la sesión ROS allí, porque
ese respaldo no tiene acceso al dispositivo.

```bash
# En Ubuntu, fuera del contenedor:
docker start ros2-humble
docker exec -it -e ROS_DOMAIN_ID=42 -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/mycobot-Xauthority ros2-humble bash
# Dentro:
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
# Solo si no hay una sesión ya abierta:
ros2 launch mycobot_280_hardware control.launch.py adaptive_gripper:=true
```

La lectura inicial de pinza cerrada (`15`) quedaba fuera del modelo con la
conversión nominal. Tras la confirmación del operador y la prueba supervisada,
se registró el valor cerrado estable `12` y se actualizaron ambos YAML. Se
verificó la parada por mantenimiento de posición durante una apertura pequeña,
sin mover las seis articulaciones del brazo. El extremo abierto sigue siendo
nominal; no se ensancharon los límites del URDF ni se recortan las mediciones.

Para una nueva sesión física en **este equipo ya comprobado**, deteniendo antes
la sesión anterior (un único propietario USB):

```bash
ros2 launch mycobot_280_hardware control.launch.py \
  adaptive_gripper:=true allow_motion:=true gripper_stop_confirmed:=true
```

El destino inicial sigue siendo Gazebo: seleccioná **Manipulador físico** en
RViz para enviar planes o seguimiento al robot. No habilites esa confirmación
en otro equipo sin comprobar su calibración y parada. La parada con carga sigue
sin validar.

El 26/10/2026 se verificaron movimientos reales mediante Plan y Execute
separados, Plan & Execute, el marcador de seguimiento y el deslizador de pinza.
La tolerancia final configurada es 0.04 rad (aproximadamente 2.3°). Esas pruebas
corresponden al seguimiento anterior. Desde el 27/10, el marcador y el deslizador
usan Plan & Execute automáticamente, con los controladores de trayectoria
activos tanto en Gazebo como en hardware. Se validó el flujo nuevo en Gazebo;
queda pendiente repetir la validación física con el USB conectado. Ver
[pruebas del flujo automático](../../docs/testing/2026-10-27-automatic-plan-execute.md).

En este M5, tras una orden de pinza sin ACK, consultar directamente los ángulos
dejaba de recibir respuesta. El transporte consulta primero una posición válida
de la pinza, bajo el mismo bloqueo USB, y después lee el brazo. Una escritura
sola no acredita ejecución ni parada. Ver [resultados y reproducción](../../docs/testing/2026-10-26-physical-control-validation.md).
