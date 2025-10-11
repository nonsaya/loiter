import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class MinimalMissionNode(Node):
    """Minimal mission: arm -> takeoff(1.0 m) -> hover(10 s) -> land -> disarm.

    NOTE: This is a skeleton for TDD/MVP. Arming/mode services and LAND handling
    are placeholders to be wired to MAVROS topics/services in subsequent edits.
    """

    def __init__(self) -> None:
        super().__init__('minimal_mission_node')

        self.declare_parameter('takeoff_altitude', 1.0)
        self.declare_parameter('hover_seconds', 10.0)

        self.takeoff_altitude = float(self.get_parameter('takeoff_altitude').value)
        self.hover_seconds = float(self.get_parameter('hover_seconds').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher placeholder: setpoint position local (MVP)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos)

        self.state = 'IDLE'
        self.state_started_at: Time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self._on_timer)  # 20 Hz

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.state_started_at).nanoseconds / 1e9

        if self.state == 'IDLE':
            # TODO: check preconditions and arm + set GUIDED via MAVROS
            self._transition('TAKEOFF')
        elif self.state == 'TAKEOFF':
            self._publish_z_setpoint(self.takeoff_altitude)
            # Placeholder convergence: wait fixed 3 s
            if elapsed >= 3.0:
                self._transition('HOVER')
        elif self.state == 'HOVER':
            self._publish_z_setpoint(self.takeoff_altitude)
            if elapsed >= self.hover_seconds:
                self._transition('LAND')
        elif self.state == 'LAND':
            # TODO: switch to LAND mode via MAVROS and wait until disarm
            if elapsed >= 3.0:
                self._transition('DONE')
        elif self.state == 'DONE':
            # No-op; keep publishing nothing
            pass

    def _transition(self, next_state: str) -> None:
        self.state = next_state
        self.state_started_at = self.get_clock().now()
        self.get_logger().info(f'state -> {next_state}')

    def _publish_z_setpoint(self, z: float) -> None:
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.z = float(z)
        self.setpoint_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MinimalMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


