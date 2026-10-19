import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class NucHeartbeatNode(Node):
    def __init__(self):
        super().__init__('nuc_heartbeat')

        self.declare_parameter('rate', 10.0)
        rate = self.get_parameter('rate').get_parameter_value().double_value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_ = self.create_publisher(Empty, '/nuc_heartbeat', qos)

        period = 1.0 / rate
        self.timer_ = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'NUC heartbeat publishing at {rate} Hz on /nuc_heartbeat')

    def timer_callback(self):
        self.pub_.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = NucHeartbeatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
