# gamora — ROS 2 / MoveIt 2

ROS 2 port of [gamora](https://github.com/abdu7rahman/gamora): a 5-DOF arm with a
two-finger gripper, its MoveIt configuration, and the nodes that drive it.

## Layout

The ROS 1 repo had two packages; this splits them three ways, which is the
conventional MoveIt 2 arrangement — geometry, configuration and behaviour each
own their package.

```
gamora_description/     URDF, xacro, meshes, rviz display
gamora_moveit_config/   SRDF, kinematics, planners, ros2_control, MoveIt launch
gamora_control/         motion nodes (moveit_py, IK following, joint state demo)
```

| ROS 1 | ROS 2 |
|---|---|
| `gamora_test_v6/urdf`, `meshes` | `gamora_description` |
| `gamora/config`, `gamora/launch` | `gamora_moveit_config` |
| `gamora_test_v6/move.py` | `gamora_control/move_to_pose` |
| `gamora_test_v6/jsptest.py` | `gamora_control/joint_state_demo` |
| `gamora_test_v6/pne_realtime.py` | `gamora_control/ik_pose_follower` |
| `gamora_test_v6/realtime_moveit.py` | `gamora_control/ik_pose_follower` |

## Build

```bash
mkdir -p ~/ros2_ws/src
cp -r gamora_description gamora_moveit_config gamora_control ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Run

Full MoveIt demo with mock hardware — plan and execute in rviz2:

```bash
ros2 launch gamora_moveit_config demo.launch.py
```

Just look at the model:

```bash
ros2 launch gamora_description display.launch.py
```

Drive to the hard-coded grasp pose (needs `demo.launch.py` running):

```bash
ros2 launch gamora_control move_to_pose.launch.py
```

Follow rviz interactive-marker drags through IK:

```bash
ros2 run gamora_control ik_pose_follower
```

## What changed from ROS 1

| ROS 1 | ROS 2 |
|---|---|
| catkin `CMakeLists.txt` | `ament_cmake` / `ament_python` |
| `package.xml` format 2 | format 3 with `<build_type>` |
| 26 `.launch` / `.launch.xml` files | 3 Python launch files via `MoveItConfigsBuilder` |
| `moveit_commander` | `moveit_py` (`MoveItPy`, `PlanningComponent`) |
| `<transmission>` + `gazebo_ros_control` | `<ros2_control>` + `gz_ros2_control` |
| `ros_control` `controllers.yaml` | `ros2_controllers.yaml` + `moveit_controllers.yaml` |
| `rospy.ServiceProxy` (blocking) | `call_async` + done callback |
| raw `.urdf` loaded directly | `.urdf.xacro` wrapper, export left untouched |

The setup-assistant launch tree — `planning_context`, `move_group`,
`moveit_rviz`, `trajectory_execution`, `*_planning_pipeline`,
`*_moveit_controller_manager`, `warehouse`, `sensor_manager` — has no
equivalent in MoveIt 2. `MoveItConfigsBuilder` assembles the same parameters in
one place, which is why 26 launch files become 3.

### Things that needed a decision, not just a translation

- **`ros_controllers.yaml` was empty.** MoveIt had no controller to execute
  against, so nothing the ROS 1 setup planned could actually run on hardware or
  in Gazebo. The port defines an `arm_controller` and a `gripper_controller`
  matching the SRDF groups.
- **`joint_names_gamora_test_v6.yaml` was wrong.** It listed `joiint3`
  (misspelled), included an empty string as the first entry, and named
  `joint_leff`/`joint_reff` alongside a `joint3` that never appeared. Joint names
  come from the URDF now; that file is gone.
- **`jsptest.py` published joints that do not exist.** It sent `joint1`..`joint6`
  while the arm has `joint1`..`joint5` plus `joint_leff`/`joint_reff`, so
  `joint6` was ignored and the gripper never moved. The names are a parameter
  defaulting to the real set.
- **The cached IK plugin is not available.** `kinematics.yaml` asked for
  `cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin`, which is not in a
  standard MoveIt 2 install. It falls back to `KDLKinematicsPlugin`; swap in
  `pick_ik` if you want the speed back.
- **`move.py` ignored planning failures.** `arm_group.go()`'s return value was
  discarded, so a failed plan was indistinguishable from a successful one. The
  port checks and logs.
- **`realtime_moveit.py` ran a full motion plan per marker drag** and then threw
  the trajectory away, publishing only its final point to `/joint_states`. That
  is what an IK call does, far more cheaply — so both scripts converge on
  `ik_pose_follower`, which is the `pne_realtime.py` approach.
- **`rospy.wait_for_service` in a constructor.** In ROS 2 that blocks the thread
  the executor has not started spinning on yet. Availability is polled on a
  timer instead.

The SRDF, joint limits and collision-disable pairs carry over unchanged. The
SolidWorks URDF export is included verbatim by the xacro wrapper, so
regenerating it from CAD will not wipe out the `ros2_control` tags.

## Note on the gripper

`joint_leff` and `joint_reff` are prismatic with a ±0.018 m range and are driven
independently. If they are meant to mirror each other, a `<mimic>` tag in the
URDF would be the right fix — the ROS 1 version did not have one either, so the
behaviour is unchanged here.
