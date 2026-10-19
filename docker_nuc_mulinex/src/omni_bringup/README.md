# omni_bringup

Launch orchestration package that brings up the NUC-side nodes for the Mulinex robot: joystick teleop, IK controller, rosbag recording, and the NUC heartbeat.

## NUC Heartbeat (Safety-Critical)

The `nuc_heartbeat` node publishes `std_msgs/Empty` at 10 Hz on `/nuc_heartbeat`. The SBC's `omni_controller` monitors this topic and triggers a **safety damping shutdown** if the heartbeat is lost for longer than `safety.heartbeat_timeout` (default 1.0 s).

**This node must always be running when the robot is active.** If the NUC crashes, the network cable is disconnected, or this node is stopped, the SBC will detect the loss and ramp the motors down safely.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rate`    | `10.0`  | Publish rate in Hz |

### Running standalone

```bash
ros2 run omni_bringup nuc_heartbeat_node --ros-args -p rate:=10.0
```

### SBC-side configuration

In the controller YAML (`omnicar.yaml` / `omniquad12.yaml`), under `omni_controller.ros__parameters.safety`:

```yaml
heartbeat_timeout: 1.0  # seconds; 0 disables the check
```

The timeout is only evaluated after the first heartbeat is received, so the SBC operates normally if the NUC has never been connected.

## Launch

```bash
ros2 launch omni_bringup omni_bringup.launch.py
```

This starts (in order): `nuc_heartbeat`, joystick teleop, IK controller, and rosbag recording.
