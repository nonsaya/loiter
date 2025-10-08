import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GlimToMavrosBridge(Node):
    def __init__(self) -> None:
        super().__init__('glim_to_mavros_bridge')
        self.declare_parameter('input_topic', '/glim/odom')
        self.declare_parameter('output_topic', '/mavros/odometry/in')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        qos_profile = rclpy.qos.QoSProfile(depth=10)

        self.pub = self.create_publisher(Odometry, self.output_topic, qos_profile)
        self.sub = self.create_subscription(Odometry, self.input_topic, self._on_odom, qos_profile)

        self.get_logger().info(
            f'Bridging {self.input_topic} -> {self.output_topic} with frame_id={self.frame_id}, child_frame_id={self.child_frame_id}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        if self.frame_id:
            out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id or msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GlimToMavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


