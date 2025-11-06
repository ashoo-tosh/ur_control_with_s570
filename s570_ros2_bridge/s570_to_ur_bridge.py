#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class S570ToURBridge(Node):
    def __init__(self):
        super().__init__('s570_to_ur_bridge')

        # Previous UR10 joint positions
        self.prev_positions = [0.0] * 6

        # Subscribe to S570 joint data
        self.subscription = self.create_subscription(
            JointState,
            '/s570_joint_states',
            self.s570_callback,
            10
        )

        # Publisher to UR10 joint states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Joint limits (radians)
        self.joint_limits = [
            (-2.0, 2.0),   # shoulder_pan_joint
            (-1.5, 1.5),   # shoulder_lift_joint
            (-2.5, 2.5),   # elbow_joint
            (-3.0, 3.0),   # wrist_1_joint
            (-2.0, 2.0),   # wrist_2_joint
            (-3.0, 3.0)    # wrist_3_joint
        ]

        self.get_logger().info('✅ S570 → UR10 Bridge Node Started (6 joints, skipping J9)')

    def s570_callback(self, msg):
        if len(msg.position) < 14:
            self.get_logger().warn('Received incomplete joint data')
            return

        # Mapping with skipped joint J9
        indices = [7, 8, 10, 11, 12, 13]  # J7, J8, J10, J11, J12, J13
        ur_positions = []
        for i, idx in enumerate(indices):
            rad = math.radians(msg.position[idx])
            lower, upper = self.joint_limits[i]
            rad = max(min(rad, upper), lower)
            ur_positions.append(rad)

        # Optional tiny smoothing
        for i in range(6):
            ur_positions[i] = self.prev_positions[i] + 0.05 * (ur_positions[i] - self.prev_positions[i])

        self.prev_positions = ur_positions

        # Publish to UR10
        ur_msg = JointState()
        ur_msg.header.stamp = self.get_clock().now().to_msg()
        ur_msg.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        ur_msg.position = ur_positions
        self.publisher_.publish(ur_msg)


def main(args=None):
    rclpy.init(args=args)
    node = S570ToURBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
