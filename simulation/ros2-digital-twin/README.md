# ROS 2 digital twin

URDF model of the gantry (exported from Onshape, post-processed by `fix_urdf.py`) with a closed-loop
control node for the X, Y and Z axes. Runs in a Podman container with ROS 2 Jazzy and Gazebo Harmonic.

```
Onshape CAD -> URDF + STL export -> fix_urdf.py -> main_assembly.urdf
                                                      |-> RViz2 (geometry and joint-limit check)
                                                      |-> Gazebo + gz_ros2_control + gantry_control node
```

## Layout

| Path | Role |
|---|---|
| `fix_urdf.py` | Post-processes the Onshape URDF: fixes mesh paths, adds `ros2_control` and Gazebo tags, collapses non-driving joints |
| `src/main_gantry_assembly/` | URDF, meshes, world, controller-manager config and launch files |
| `src/gantry_control/` | `ament_python` package: three SISO loops, motor and sensor models, difference-equation block |

## Run

Inside a container based on `osrf/ros:jazzy-desktop` with this folder mounted at `/workspace`:

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace && colcon build --symlink-install && source install/setup.bash

# geometry check, joints driven by sliders, no physics
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch main_gantry_assembly main_gantry_assembly.launch.py

# Gazebo physics with the control node (add use_rviz:=true for RViz)
ros2 launch main_gantry_assembly gantry_gazebo_control.launch.py
ros2 topic pub /gantry/setpoint std_msgs/msg/Float64MultiArray "{data: [200.0, 300.0, 50.0]}" -1   # mm
```

## Status and limitations

- The RViz twin works. The Gazebo control path is wired but was **not run as a test** for the final design,
  so treat it as a starting point and not as a validated result.
- The control node uses the unloaded-motor speed model, not the loaded plants identified on the built gantry.
  Those loaded plants and their controller design are in `../controller-design/`.
- DART (the Gazebo default) does not enforce mimic joints. The one that mattered for control was collapsed to a fixed joint.
- No joint friction is declared in the URDF, so the simulated plant is frictionless unless you add `<dynamics>` tags.
- The Onshape assembly has loop-closure joints. URDF has no closed loops, so the export breaks them into tree branches.
