# Portable Docker environment

**Goal:** A new Linux PC can build the ROS 2 Humble environment and start Gazebo
and RViz without the original workstation's container or absolute paths.

**Scope approved in conversation:** Dockerfile, Compose, graphics/USB setup and
README commands. Work in the existing checkout to include its current robot code.

**Design:** Build from OSRF's Humble desktop-full Jammy image. Copy and compile
the current packages into `/opt/mycobot_ws`; fetch the same warehouse commit used
by the existing workspace. Start simulation without USB or physical permission.
Use an explicit Compose override for USB and another for NVIDIA acceleration.
Use X11 authentication, host networking and separate ROS/Gazebo defaults to avoid
interfering with the existing running workstation session. Source changes require
an image rebuild. No publishing or changes to the existing container.

**Alternatives:** Exporting the workstation container is large and opaque;
installing ROS on each host defeats the requested shared environment.

## Implementation and checks

- [x] Add Dockerfile, build-context exclusions and entrypoint. Build from OSRF
  without depending on the existing container's filesystem or installed packages.
- [x] Add Compose simulation, hardware and NVIDIA configurations. Validate all
  combinations, including a custom USB path; keep physical motion opt-in.
- [x] Rewrite the README quick start for a new clone, retain workstation notes,
  and document X11, rebuilding, shutdown and resource use.
- [x] Verify the image's ROS package discovery, software tests and a headless
  Gazebo/MoveIt simulation in an isolated domain; document actual limitations.

**Review focus:** Missing USB must not block simulation. Missing X11 credentials
must not silently create a directory. Overrides must preserve motion defaults.
No host-specific paths or source mounts that hide compiled packages. Rebuilding
must incorporate this checkout's current uncommitted robot code.

**Result:** See `docs/testing/2026-10-27-portable-docker.md`. Read-only review
identified hostname-specific X11 authentication; Compose now preserves the
authorization hostname exported by the host. All image tests passed.
