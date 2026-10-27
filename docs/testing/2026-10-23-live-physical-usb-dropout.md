# Live physical control: isolated missing USB sample

Historical diagnosis. Subsequent real movements and the gripper USB fix are
documented in the [26 October validation](2026-10-26-physical-control-validation.md).
That report supersedes the pending physical verification and 0.5 s runtime
deadline described below.

Following the report that real live movement behaved badly or stopped, the
session log showed live activation at ROS time 1792785181.6559, rejection of a
stale/inactive command at 1792785182.7146 and a missing/invalid gripper reading at
1792785183.4741. The robot remained USB-connected. Later simulation logs reported
MoveIt Servo's singularity stop. These observations identify separate failure
paths; they do not establish that all reported movement problems are resolved.

## Reproduced defects

A complete hardware sample was followed by an incomplete sample. The driver
immediately erased the valid measurement timestamp and stopped the session,
instead of allowing the existing 0.5-second freshness deadline to expire.

A verified stop then published only “Brazo USB conectado,” hiding its reason.

Both regression tests failed before implementation. The driver now retains the
last complete sample and its original timestamp for missing samples, without
renewing freshness or synthesizing missing values. A complete out-of-bounds
sample still invalidates the state immediately. The stop reason is logged and
remains visible until a new physical selection succeeds.

Functional suite: **135 passed, 3 inherited linters deselected**, with the existing
xacro warning. Log: `/tmp/mycobot-live-usb-fix-tests.log` inside `ros2-humble`.
Ruff F/E9 and `git diff --check` passed.

No physical motion was commanded during diagnosis or verification. Runtime was
restarted into Gazebo with the existing calibrated hardware settings and the
physical session unselected. Singularity thresholds, physical speed limits,
freshness deadline, command ordering and stop checks were not relaxed. A nearly
straight arm can still halt Cartesian live tracking; a planned bent posture is
needed before using that tracking mode. Incorrect direction or axis mapping
requires separate observation and has not been assumed corrected here.

The operator clarified: “El brazo casi no se mueve.” A read-only geometric
Jacobian calculation using the expanded gripper URDF and measured physical
positions gave condition **5712.51**, above the configured Servo hard-stop
threshold **200**. The existing `ready` named posture gave **88.18** in the same
calculation. This supports singularity braking as the reason for minimal
Cartesian movement from the current almost-straight physical pose. The calculation
commanded no motion and does not validate the physical clearance of a trajectory.

Operator sequence: turn off live tracking, select the physical destination,
select `arm_group` and Goal State `ready` in MotionPlanning, Plan and inspect the
preview, then Execute if the path is clear. Enable live tracking after reaching
the bent posture. Do not increase singularity thresholds to force the straight
posture to track. Full physical following still needs operator verification.

Updated session log: `/tmp/mycobot-unified-physical-usb-fixed.log`.
