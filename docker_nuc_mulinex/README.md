# Docker NUC Mulinex

This repository contains the Docker configuration for the NUC of the Mulinex project.

## Initial Setup of the Machine

For the initial setup of the NUC and of the Raspberry, refer to the **omnicar** section in [mulinex_guide](https://github.com/CentroEPiaggio/mulinex_guide).

## Preliminaries

Clone the repository with the `--recursive` option to also clone the submodules!

Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine).
You can follow the installation method through `apt`.
Note that it makes you verify the installation by running `sudo docker run hello-world`.
It is better to avoid running this command with `sudo` and instead follow the post installation steps first and then run the command without `sudo`.

Follow with the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for Linux.
This will allow you to run Docker without `sudo`.

## Docker

Build the docker image (use the `-r` option to update the underlying images):
```shell
./docker/build.bash [-r]
```

Run the container:
```shell
./docker/run.bash
```

### Docker Compose

As an alternative to `build.bash` and `run.bash`, you can use Docker Compose.

Build, start, attach, and stop the container with `make build`, `make start`, `make attach`, and `make stop` respectively.


## Usage

Connect to the Raspberry with
```shell
ssh mulsbc@100.100.100.3
```
The password is `123456`.

The Raspberry will print a message confirming if chrony is synchronized or not. If it is not synchronized, run on the **NUC** and **outside** of the **container**
```shell
sudo systemctl restart chrony
```
to synchronize it. You can check the synchronization status with
```shell
check_chrony_100
```

### Launch Files

Optional arguments are between square brackets `[]`. Multiple options are separated by a pipe `|` and the first value is the default.

Start the hardware interface on the Raspberry.
The updated hardware interface launch file automatically starts the necessary controllers.

Activate the joystick with
```shell
ros2 launch omni_bringup omni_bringup.launch.py [record_bag:={false|true}]
```

Press `SELECT` on the joystick to start a scripted **demo cycle** that drives the robot through a fixed sequence of base motions (translations + yaw) at rest height, then at standing height, and finally exercises the IK controller through pitch, roll, and base translations before lowering back to rest. Press `SELECT` again at any time to interrupt: the robot resets its pose, deactivates IK, and returns to rest. The full button mapping is printed in the joystick node log on startup.

Launch the lidar with
```shell
ros2 launch sllidar_ros2 view_sllidar_s2e_launch.py device_ip:=192.168.11.2
```

### Gazebo Simulation

Launch the Gazebo simulation with
```shell
ros2 launch omni_gazebo gz_sim.launch.py [robot:={omnicar|omniquad12}] [world:=empty.sdf] [gz_verbosity:={0|1|2|3|4}] [use_rviz:={false|true}]
```

The launch file starts Ignition Gazebo, spawns the selected robot, brings up the ROS↔Gazebo bridge (`/clock`, `/imu`), and spawns the `joint_state_broadcaster` and `omni_controller` ros2_control controllers.

| Argument | Default | Description |
|---|---|---|
| `robot` | `omnicar` | Robot model to simulate (`omnicar` or `omniquad12`) |
| `world` | `empty.sdf` | World SDF file, looked up in `omni_gazebo/worlds/` |
| `gz_verbosity` | `3` | Gazebo verbosity level (0 = silent, 4 = verbose) |
| `use_rviz` | `false` | Also launch RViz2 with the robot's default config |

Example — simulate the Mulinex quadruped with RViz open:
```shell
ros2 launch omni_gazebo gz_sim.launch.py robot:=omniquad12 use_rviz:=true
```

## Packages

- **`ik_controller`** - Inverse kinematics controller using Pinocchio that computes joint positions from desired base pose commands, keeping foot positions fixed in the world frame.
- **`mulinex_description`** - Robot URDF/Xacro description package providing kinematic and dynamic models for the Mulinex quadruped and OmniCar wheeled variants.
- **`omni_bringup`** - Launch orchestration package that brings up the joystick teleop and IK controller nodes while recording key system topics to rosbag.
- **`omni_mulinex_joystick`** - PS4 joystick teleoperation node that maps controller inputs to twist velocity commands and base pose targets for the omnidirectional robot.
- **`omni_utils`** - Shared header-only C++ utility library providing quaternion mathematics functions used by teleoperation and control nodes.
- **`omnicar_test`** - Test package containing trajectory generation utilities and linear trajectory planning for validation and simulation of robot motion.
- **`pi3hat_moteus_int_msgs`** - Custom ROS 2 message definitions for pi3hat/moteus motor control communication (JointsCommand, JointsStates, OmniMulinexCommand, etc.).
- **`robot_model`** - Pinocchio-based C++ and Python wrapper providing kinematics and dynamics computation for quadrupedal robots, loading configs from YAML and URDF. **Should be used for kinematics and dynamics computations across controllers.**
- **`sllidar_ros2`** - ROS 2 driver for SLLIDAR LiDAR sensors that publishes laser scan data for mapping and localization.
- **`teleop_mulinex`** - Keyboard teleoperation node for controlling the Mulinex robot with incremental adjustments to velocity and body pose. Implements the same functionality as the joystick teleop.

## Architecture

`omni_mulinex_joystick` publishes:
- `/omni_controller/twist_cmd` [`geometry_msgs/Twist`]: base velocity commands (linear x, y and angular z) from the joystick
- `/ik_controller/base_pose` [`geometry_msgs/Pose`]: body pose commands (position and orientation) from the joystick

---

`ik_controller` subscribes to:
- `/ik_controller/base_pose` [`geometry_msgs/Pose`]: desired body pose from the joystick
- `/omni_controller/joints_state` [`pi3hat_moteus_int_msgs/JointsStates`]: current joint states from the hardware interface

and publishes:
- `/omni_controller/joints_reference` [`pi3hat_moteus_int_msgs/JointsCommand`]: reference setpoints (position/velocity/effort) for non-wheel joints computed via inverse kinematics

---

The hardware interface (`omni_controller`) subscribes to:
- `/omni_controller/twist_cmd` [`geometry_msgs/Twist`]: wheel velocity commands from the joystick
- `/omni_controller/joints_reference` [`pi3hat_moteus_int_msgs/JointsCommand`]: reference setpoints for non-wheel joints (legs, arms, etc.) from the IK controller

and publishes:
- `/omni_controller/joints_state` [`pi3hat_moteus_int_msgs/JointsStates`]: joint feedback (position, velocity, effort, current, temperature)
- `/omni_controller/debug/joints_command` [`pi3hat_moteus_int_msgs/JointsCommand`]: echo of the commands actually written to hardware each cycle (for debugging / logging)
- `/omni_controller/odom` [`geometry_msgs/TwistStamped`]: odometry from wheel forward kinematics (optional)
- `/omni_controller/performance` [`pi3hat_moteus_int_msgs/PacketPass`]: communication performance metrics (optional)
- `/omni_controller/distributors_state` [`pi3hat_moteus_int_msgs/DistributorsState`]: power distributor state (optional)

## Acknowledgements

- Attila De Benedittis
- Lorenz1 Martignetti
- Jacopo **Tiffany** Cioni (for grammatical errors)