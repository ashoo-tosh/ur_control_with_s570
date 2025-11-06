# ur_control_with_s570
# S570 → UR10 Bridge

## Project Overview

The **S570 → UR10 Bridge** is a ROS 2 package designed to map the joint positions of an **S570 robotic arm** to a **UR10 robotic arm** in real-time. The main goal is to replicate the motion of the S570 arm in RViz or any compatible UR10 simulation with minimal delay and accurate joint mapping.

**Features:**

* Real-time teleoperation of UR10 using S570 joint inputs.
* Instant feedback visualization in RViz.
* Safe and controlled motion through joint limits.

---

## ROS Topics

### Subscribed Topics

| Topic                | Type                     | Description                                         |
| -------------------- | ------------------------ | --------------------------------------------------- |
| `/s570_joint_states` | `sensor_msgs/JointState` | Receives joint positions from the S570 robotic arm. |

### Published Topics

| Topic           | Type                     | Description                                                                        |
| --------------- | ------------------------ | ---------------------------------------------------------------------------------- |
| `/joint_states` | `sensor_msgs/JointState` | Publishes mapped joint positions to the UR10 for RViz visualization or simulation. |

---

## Joint Mapping

| UR10 Joint            | S570 Source Joint | Notes                                        |
| --------------------- | ----------------- | -------------------------------------------- |
| `shoulder_pan_joint`  | J7                | Optional softening applied to reduce jitter. |
| `shoulder_lift_joint` | J8                | Direct mapping.                              |
| `elbow_joint`         | J10               | Skips J9 to maintain correct motion.         |
| `wrist_1_joint`       | J11               | Direct mapping.                              |
| `wrist_2_joint`       | J12               | Direct mapping.                              |
| `wrist_3_joint`       | J13               | Direct mapping.                              |

---

## Installation & Setup

1. Ensure **ROS 2 Humble** is installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

2. Clone the ROS 2 package into your workspace:

```bash
cd ~/s570_ros2_ws/src
git clone <repository_url>
```

3. Build the workspace:

```bash
cd ~/s570_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

4. Connect the S570 arm via USB (`/dev/ttyACM0`).

5. Launch the bridge node:

```bash
ros2 run s570_ros2_bridge s570_to_ur_bridge
```

6. Open RViz to visualize the UR10 robot:

```bash
ros2 run rviz2 rviz2
```

* Add a **RobotModel** display.
* Set the topic to `/joint_states`.

---

## Node Implementation

* **Node Name:** `s570_to_ur_bridge`
* **Programming Language:** Python 3
* **Key Components:**

  * Subscriber to `/s570_joint_states`
  * Publisher to `/joint_states`
  * Direct joint mapping
  * Optional smoothing for `shoulder_pan_joint`
  * Enforces joint limits

**Joint indices mapping in code:**

```python
# Mapping indices from S570 to UR10
indices = [7, 8, 10, 11, 12, 13]
```

---

## Joint Limits

| Joint               | Min (rad) | Max (rad) |
| ------------------- | --------- | --------- |
| shoulder_pan_joint  | -2.0      | 2.0       |
| shoulder_lift_joint | -1.5      | 1.5       |
| elbow_joint         | -2.5      | 2.5       |
| wrist_1_joint       | -3.0      | 3.0       |
| wrist_2_joint       | -2.0      | 2.0       |
| wrist_3_joint       | -3.0      | 3.0       |

---

## Optional Improvements

* **Softening factor** for shoulder_pan_joint to reduce jitter.
* **Rate limiting** to prevent sudden jumps in robot motion.
* **Logging & diagnostics** for debugging.

---

## Troubleshooting

| Issue                     | Possible Cause                         | Solution                                                        |
| ------------------------- | -------------------------------------- | --------------------------------------------------------------- |
| Shoulder joint fluctuates | Minor jitter in S570 readings          | Apply softening factor (0.01–0.02)                              |
| Elbow joint not moving    | Wrong mapping                          | Ensure J10 → elbow mapping is active                            |
| No movement in RViz       | Incorrect topic or missing robot model | Check `/joint_states` topic and load correct UR10 model in RViz |
| Delay in motion           | Too low update rate                    | Increase subscriber queue or timer frequency                    |

---

## References

* ROS 2 Humble Documentation: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
* `sensor_msgs/JointState` message: [http://docs.ros2.org/humble/api/sensor_msgs/msg/JointState.html](http://docs.ros2.org/humble/api/sensor_msgs/msg/JointState.html)
* UR10 URDF Model for RViz visualization.

---

## Author

**Your Name / Team Name**
Email: [ashutoshr980@gmail.com](mailto:your.email@example.com)
Date: 2025-11-06

---

This README provides a complete overview, setup instructions, and mapping details for the S570 → UR10 Bridge project.
