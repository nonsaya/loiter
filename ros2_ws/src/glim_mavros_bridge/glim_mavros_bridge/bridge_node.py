import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rclpy.time import Time
import math


class GlimToMavrosBridge(Node):
    def __init__(self) -> None:
        super().__init__('glim_to_mavros_bridge')
        self.declare_parameter('input_topic', '/glim/odom')
        self.declare_parameter('output_topic', '/mavros/odometry/in')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('override_stamp_with_now', True)
        self.declare_parameter('publish_vision_topics', False)
        self.declare_parameter('mirror_to_odometry_out', True)
        # Diagonal covariance defaults: [pos_x, pos_y, pos_z, roll, pitch, yaw]
        self.declare_parameter('pose_cov_diag', [0.02, 0.02, 0.05, 0.01, 0.01, 0.02])
        # Diagonal covariance defaults: [vx, vy, vz, wx, wy, wz]
        self.declare_parameter('twist_cov_diag', [0.05, 0.05, 0.10, 0.02, 0.02, 0.02])

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.override_stamp_with_now = self.get_parameter('override_stamp_with_now').get_parameter_value().bool_value
        self.publish_vision_topics = self.get_parameter('publish_vision_topics').get_parameter_value().bool_value
        self.mirror_to_odometry_out = self.get_parameter('mirror_to_odometry_out').get_parameter_value().bool_value
        self.pose_cov_diag = [float(x) for x in self.get_parameter('pose_cov_diag').get_parameter_value().double_array_value]
        self.twist_cov_diag = [float(x) for x in self.get_parameter('twist_cov_diag').get_parameter_value().double_array_value]

        qos_profile = rclpy.qos.QoSProfile(depth=10)

        self.pub = self.create_publisher(Odometry, self.output_topic, qos_profile)
        # VISION route publishers (optional)
        self.pub_pose_cov = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', qos_profile)
        self.pub_twist = self.create_publisher(TwistStamped, '/mavros/vision_speed/speed_twist', qos_profile)
        # Mirror publisher for debugging/visibility
        self.pub_mirror_out = self.create_publisher(Odometry, '/mavros/odometry/out', qos_profile)
        self.sub = self.create_subscription(Odometry, self.input_topic, self._on_odom, qos_profile)

        # state for velocity estimation
        self._last_time: Time | None = None
        self._last_xyz: tuple[float, float, float] | None = None

        self.get_logger().info(
            f'Bridging {self.input_topic} -> {self.output_topic} with frame_id={self.frame_id}, child_frame_id={self.child_frame_id}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        if self.override_stamp_with_now:
            out.header.stamp = self.get_clock().now().to_msg()
        if self.frame_id:
            out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id or msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        # Estimate linear velocity if incoming twist is zero vector
        try:
            vx = out.twist.twist.linear.x
            vy = out.twist.twist.linear.y
            vz = out.twist.twist.linear.z
            twist_all_zero = (abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(vz) < 1e-6)
        except Exception:
            twist_all_zero = True

        try:
            px = out.pose.pose.position.x
            py = out.pose.pose.position.y
            pz = out.pose.pose.position.z
        except Exception:
            px = py = pz = 0.0

        now_t = self.get_clock().now()
        if twist_all_zero and self._last_time is not None and self._last_xyz is not None:
            dt = (now_t.nanoseconds - self._last_time.nanoseconds) / 1e9
            if dt > 1e-3:
                vx_est = (px - self._last_xyz[0]) / dt
                vy_est = (py - self._last_xyz[1]) / dt
                vz_est = (pz - self._last_xyz[2]) / dt
                out.twist.twist.linear.x = vx_est
                out.twist.twist.linear.y = vy_est
                out.twist.twist.linear.z = vz_est

        # update state
        self._last_time = now_t
        self._last_xyz = (px, py, pz)

        # Convert linear velocity from world(ENU) to body frame if possible (EKF expects body-frame by default)
        try:
            q = out.pose.pose.orientation
            # rotation matrix R from body->world; we need world->body so use R^T
            # quaternion normalization
            norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z) or 1.0
            qw, qx, qy, qz = q.w/norm, q.x/norm, q.y/norm, q.z/norm
            # R (body->world)
            r00 = 1 - 2*(qy*qy + qz*qz)
            r01 = 2*(qx*qy - qz*qw)
            r02 = 2*(qx*qz + qy*qw)
            r10 = 2*(qx*qy + qz*qw)
            r11 = 1 - 2*(qx*qx + qz*qz)
            r12 = 2*(qy*qz - qx*qw)
            r20 = 2*(qx*qz - qy*qw)
            r21 = 2*(qy*qz + qx*qw)
            r22 = 1 - 2*(qx*qx + qy*qy)
            # v_body = R^T * v_world
            vw_x = out.twist.twist.linear.x
            vw_y = out.twist.twist.linear.y
            vw_z = out.twist.twist.linear.z
            vb_x = r00*vw_x + r10*vw_y + r20*vw_z
            vb_y = r01*vw_x + r11*vw_y + r21*vw_z
            vb_z = r02*vw_x + r12*vw_y + r22*vw_z
            out.twist.twist.linear.x = vb_x
            out.twist.twist.linear.y = vb_y
            out.twist.twist.linear.z = vb_z
        except Exception as e:
            self.get_logger().warn(f"velocity transform skipped: {e}")

        # If covariance is all zeros, provide small reasonable diagonals so EKF can weight the measurement
        try:
            if all(c == 0.0 for c in out.pose.covariance):
                pose_cov = list(out.pose.covariance)
                # x,y,z, roll,pitch,yaw on diagonals
                for i, v in enumerate(self.pose_cov_diag):
                    pose_cov[i * 6 + i] = v
                out.pose.covariance = pose_cov
            if all(c == 0.0 for c in out.twist.covariance):
                twist_cov = list(out.twist.covariance)
                # vx,vy,vz, wx,wy,wz
                for i, v in enumerate(self.twist_cov_diag):
                    twist_cov[i * 6 + i] = v
                out.twist.covariance = twist_cov
        except Exception as e:
            self.get_logger().warn(f"covariance fix skipped: {e}")
        self.pub.publish(out)
        if self.mirror_to_odometry_out:
            self.pub_mirror_out.publish(out)

        if self.publish_vision_topics:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = out.header
            pose_msg.pose = out.pose
            self.pub_pose_cov.publish(pose_msg)

            tw_msg = TwistStamped()
            tw_msg.header = out.header
            tw_msg.twist = out.twist.twist
            self.pub_twist.publish(tw_msg)


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


