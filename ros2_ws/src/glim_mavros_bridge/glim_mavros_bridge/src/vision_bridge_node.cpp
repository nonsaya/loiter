// Minimal GLIM -> MAVROS vision bridge
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class VisionBridgeNode : public rclcpp::Node {
public:
  VisionBridgeNode() : rclcpp::Node("glim_mavros_vision_bridge") {
    input_pose_topic_ = this->declare_parameter<std::string>(
        "input_pose_topic", "/glim/pose");
    output_pose_topic_ = this->declare_parameter<std::string>(
        "output_pose_topic", "/mavros/vision_pose/pose");

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        output_pose_topic_, rclcpp::QoS(30));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        input_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&VisionBridgeNode::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Bridging '%s' -> '%s'",
                input_pose_topic_.c_str(), output_pose_topic_.c_str());
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Forward as-is. MAVROS handles ENU->NED conversion internally.
    geometry_msgs::msg::PoseStamped out = *msg;
    // Ensure frame_id exists; default to "map" which GLIM uses.
    if (out.header.frame_id.empty()) {
      out.header.frame_id = "map";
    }
    pose_pub_->publish(out);
  }

  std::string input_pose_topic_;
  std::string output_pose_topic_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


