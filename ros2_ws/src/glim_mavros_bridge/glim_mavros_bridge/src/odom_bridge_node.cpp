#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <string>
#include <memory>
#include <optional>

using namespace std::chrono_literals;

class OdomBridgeNode : public rclcpp::Node {
public:
  OdomBridgeNode() : rclcpp::Node("glim_mavros_odometry_bridge") {
    // Parameters
    glim_namespace_ = this->declare_parameter<std::string>("glim_namespace", "/glim");
    use_corrected_ = this->declare_parameter<bool>("use_corrected", false);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 15.0);
    odom_child_frame_id_ = this->declare_parameter<std::string>("odom_child_frame_id", "");
    restamp_source_ = this->declare_parameter<std::string>("restamp_source", "now"); // none|arrival|now
    reject_older_than_ms_ = this->declare_parameter<double>("reject_older_than_ms", 200.0);
    publish_immediately_ = this->declare_parameter<bool>("publish_immediately", true);
    target_topic_ = this->declare_parameter<std::string>("target_topic", "/mavros/odometry/out");

    // Topics
    const std::string in_topic = glim_namespace_ + (use_corrected_ ? "/odom_corrected" : "/odom");
    const std::string out_topic = target_topic_;

    // Publisher (Reliable, keep last 10)
    rclcpp::QoS qos_out(rclcpp::KeepLast(10));
    qos_out.reliability(rclcpp::ReliabilityPolicy::Reliable);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(out_topic, qos_out);

    // Subscriber (default QoS)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      in_topic, rclcpp::QoS(10),
      std::bind(&OdomBridgeNode::odomCallback, this, std::placeholders::_1));

    // Timer for rate control if not publishing immediately
    if (!publish_immediately_) {
      const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&OdomBridgeNode::timerPublish, this));
    }

    RCLCPP_INFO(get_logger(), "ODOM bridge: '%s' -> '%s' (use_corrected=%s, restamp=%s, rate=%.1f Hz)",
                in_topic.c_str(), out_topic.c_str(), use_corrected_ ? "true" : "false",
                restamp_source_.c_str(), publish_rate_hz_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const rclcpp::Time now = this->now();

    if (shouldRejectForStaleness(now, msg->header.stamp)) {
      // stale input when restamp_source==none and freshness required
      return;
    }

    last_arrival_time_ = now;
    latest_msg_ = *msg;

    if (publish_immediately_) {
      publish(*latest_msg_);
    }
  }

  void timerPublish() {
    if (!latest_msg_.has_value()) {
      return;
    }
    publish(*latest_msg_);
  }

  void publish(nav_msgs::msg::Odometry msg) {
    // Restamp policy
    if (restamp_source_ == "arrival") {
      msg.header.stamp = last_arrival_time_;
    } else if (restamp_source_ == "now") {
      msg.header.stamp = this->now();
    } // else "none": keep original

    // child_frame_id override if provided
    if (!odom_child_frame_id_.empty()) {
      msg.child_frame_id = odom_child_frame_id_;
    }

    // Basic frame consistency default (if missing)
    if (msg.header.frame_id.empty()) {
      msg.header.frame_id = "odom"; // GLIM default
    }

    odom_pub_->publish(msg);
  }

  bool shouldRejectForStaleness(const rclcpp::Time& now, const rclcpp::Time& source_stamp) const {
    if (restamp_source_ != "none") {
      return false;
    }
    if (reject_older_than_ms_ <= 0.0) {
      return false;
    }
    const double age_ms = (now - source_stamp).seconds() * 1000.0;
    return age_ms > reject_older_than_ms_;
  }

  // Params
  std::string glim_namespace_;
  bool use_corrected_{};
  double publish_rate_hz_{};
  std::string odom_child_frame_id_;
  std::string restamp_source_;
  double reject_older_than_ms_{};
  bool publish_immediately_{};
  std::string target_topic_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  rclcpp::Time last_arrival_time_{};
  std::optional<nav_msgs::msg::Odometry> latest_msg_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomBridgeNode>());
  rclcpp::shutdown();
  return 0;
}


