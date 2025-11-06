import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymycobot import Exoskeleton

class S570Publisher(Node):
    def __init__(self):
        super().__init__('s570_joint_publisher')
        
        # Publisher
        self.publisher_ = self.create_publisher(JointState, '/s570_joint_states', 10)
        
        # Connect to exoskeleton
        try:
            self.exo = Exoskeleton(port='/dev/ttyACM0')
            self.get_logger().info('Connected to S570 on /dev/ttyACM0')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            exit(1)
        
        # Timer
        self.timer = self.create_timer(0.05, self.publish_joint_state)

    def publish_joint_state(self):
        try:
            all_data = self.exo.get_all_data()
            left_arm = [float(angle) for angle in all_data[0][:7]]
            right_arm = [float(angle) for angle in all_data[1][:7]]

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [
                'left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4',
                'left_joint_5', 'left_joint_6', 'left_joint_7',
                'right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4',
                'right_joint_5', 'right_joint_6', 'right_joint_7'
            ]
            msg.position = left_arm + right_arm
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error reading joints: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = S570Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
