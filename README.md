# IR_g27_assignment_2

A ROS2 package that uses a **UR5 robotic arm** with a **Robotiq 85 gripper** to
autonomously swap the positions of two cubes in a Gazebo simulation. Each cube
has an **AprilTag** attached for detection. The camera detects both tags via a
fixed RGB camera, plans pick-and-place motions using **MoveIt Task Constructor
(MTC)**, and executes a two-phase swap sequence.

---

## System Requirements

| Item | Version |
|------|---------|
| ROS2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8) |
| MoveIt2 | main |
| OS | Ubuntu 24.04 |

---

## Installing MoveIt Task Constructor

### Standard Installation
Using the **Getting Started** section from https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html to install MTC. Move into your workspace source directory and clone the repository:

```bash
cd ~/your_ws/src
git clone -b ros2 https://github.com/moveit/moveit_task_constructor.git
```

Install missing dependencies with rosdep:

```bash
cd ~/your_ws/src
rosdep install --from-paths . --ignore-src --rosdistro jazzy
```

Build the workspace:

```bash
cd ~/your_ws
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
colcon build --mixin release
source install/setup.bash
```

### Fix: `warehouse_ros_mongo` rosdep Error

If you get the following error when running `rosdep install`:

```
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
ir_movit_config: Cannot locate rosdep definition for [warehouse_ros_mongo]
```

`warehouse_ros_mongo` is a MongoDB-based warehouse backend for MoveIt that has
no pre-built binary for Jazzy. Skip it with the `--skip-keys` flag:

```bash
rosdep install --from-paths . --ignore-src --rosdistro jazzy --skip-keys "warehouse_ros_mongo"
```

Then build and source as normal:

```bash
cd ~/your_ws
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
colcon build --mixin release
source install/setup.bash
```

Verify the installation:

```bash
ros2 pkg list | grep moveit_task_constructor
```

You should see:

```
moveit_task_constructor_capabilities
moveit_task_constructor_core
moveit_task_constructor_msgs
moveit_task_constructor_visualization
```

---

## Downloading and Building the Package

### Step 1 — Clone the repository

```bash
cd ~/your_ws/src
git clone https://github.com/moamen-ab/IR_g27_assignment_2.git
```

### Step 2 — Build the package


```bash
cd ~/your_ws
colcon build --packages-select g27_assignment_2
source install/setup.bash
```

---

## Launching the Project

```bash
ros2 launch g27_assignment_2 main.launch.py
```
