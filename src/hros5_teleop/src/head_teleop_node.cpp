#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class HeadTeleopNode : public rclcpp::Node
{
public:
  HeadTeleopNode()
  : Node("head_teleop_node")
  {
    // Parameters (can be overridden in YAML)
    pan_axis_   = this->declare_parameter<int>("pan_axis", 0);   // PS4: left stick horizontal
    tilt_axis_  = this->declare_parameter<int>("tilt_axis", 1);  // PS4: left stick vertical
    invert_pan_ = this->declare_parameter<bool>("invert_pan", false);
    invert_tilt_= this->declare_parameter<bool>("invert_tilt", true);

    pan_max_deg_  = this->declare_parameter<double>("pan_max_deg", 60.0);   // TODO: set to real limit
    tilt_max_deg_ = this->declare_parameter<double>("tilt_max_deg", 30.0);  // TODO: set to real limit
    deadzone_     = this->declare_parameter<double>("deadzone", 0.05);
    center_button_= this->declare_parameter<int>("center_button", 3);       // PS4: triangle by default

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);

    head_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/hros5/head/target_angles_deg", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&HeadTeleopNode::joyCallback, this, std::placeholders::_1));

    last_pan_deg_ = 0.0;
    last_tilt_deg_ = 0.0;

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&HeadTeleopNode::publishCommand, this));

    RCLCPP_INFO(get_logger(), "HeadTeleopNode started.");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (pan_axis_ >= 0 && static_cast<size_t>(pan_axis_) < msg->axes.size()) {
      double raw = msg->axes[pan_axis_];
      if (std::fabs(raw) < deadzone_) raw = 0.0;
      double sign = invert_pan_ ? -1.0 : 1.0;
      last_pan_deg_ = sign * raw * pan_max_deg_;
    }

    if (tilt_axis_ >= 0 && static_cast<size_t>(tilt_axis_) < msg->axes.size()) {
      double raw = msg->axes[tilt_axis_];
      if (std::fabs(raw) < deadzone_) raw = 0.0;
      double sign = invert_tilt_ ? -1.0 : 1.0;
      last_tilt_deg_ = sign * raw * tilt_max_deg_;
    }

    // Center head when center_button is pressed
    if (center_button_ >= 0 &&
        static_cast<size_t>(center_button_) < msg->buttons.size() &&
        msg->buttons[center_button_] == 1)
    {
      last_pan_deg_ = 0.0;
      last_tilt_deg_ = 0.0;
    }
  }

  void publishCommand()
  {
    std_msgs::msg::Float32MultiArray cmd;
    cmd.data.resize(2);
    cmd.data[0] = static_cast<float>(last_pan_deg_);
    cmd.data[1] = static_cast<float>(last_tilt_deg_);
    head_cmd_pub_->publish(cmd);
  }

  // Parameters
  int pan_axis_;
  int tilt_axis_;
  bool invert_pan_;
  bool invert_tilt_;
  double pan_max_deg_;
  double tilt_max_deg_;
  double deadzone_;
  int center_button_;
  double publish_rate_hz_;

  // State
  double last_pan_deg_;
  double last_tilt_deg_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr head_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
