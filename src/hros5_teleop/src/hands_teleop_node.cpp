#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace
{
double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}
}  // namespace

class HandsTeleopNode : public rclcpp::Node
{
public:
  HandsTeleopNode()
  : Node("hands_teleop_node")
  {
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "joy");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);
    fallback_dt_ = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.02;

    init_hand_params(left_hand_, "left");
    enable_right_hand_ = this->declare_parameter<bool>("enable_right_hand", false);
    if (enable_right_hand_) {
      init_hand_params(right_hand_, "right");
      right_hand_.enabled = true;
    }

    left_hand_.enabled = true;

    left_hand_.publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      left_hand_.topic, 10);
    if (right_hand_.enabled) {
      right_hand_.publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        right_hand_.topic, 10);
    }

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10, std::bind(&HandsTeleopNode::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "HandsTeleopNode started. Listening on %s", joy_topic_.c_str());
  }

private:
  struct HandParams
  {
    bool enabled{false};
    std::string topic;

    int wrist_axis{2};
    bool wrist_invert{false};
    double wrist_speed_deg_per_sec{90.0};
    double wrist_min_deg{-90.0};
    double wrist_max_deg{90.0};
    double wrist_deadzone{0.05};

    int grip_close_button{4};  // L1
    int grip_open_button{6};   // L2 (treated as digital)
    double grip_step_deg{2.0};
    double grip_min_deg{-30.0};
    double grip_max_deg{30.0};

    double wrist_deg{0.0};
    double grip_deg{0.0};

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
  };

  void init_hand_params(HandParams & hand, const std::string & prefix)
  {
    hand.topic = this->declare_parameter<std::string>(
      prefix + "_topic",
      prefix == "left"
        ? "/hros5/left_hand/target_angles_deg"
        : "/hros5/right_hand/target_angles_deg");

    hand.wrist_axis = this->declare_parameter<int>(prefix + "_wrist_axis", 0);
    hand.wrist_invert = this->declare_parameter<bool>(prefix + "_wrist_invert", false);
    hand.wrist_speed_deg_per_sec = this->declare_parameter<double>(
      prefix + "_wrist_speed_deg_per_sec", 90.0);
    hand.wrist_min_deg = this->declare_parameter<double>(prefix + "_wrist_min_deg", -90.0);
    hand.wrist_max_deg = this->declare_parameter<double>(prefix + "_wrist_max_deg", 90.0);
    hand.wrist_deadzone = this->declare_parameter<double>(prefix + "_wrist_deadzone", 0.05);

    hand.grip_close_button = this->declare_parameter<int>(prefix + "_grip_close_button", 4);
    hand.grip_open_button = this->declare_parameter<int>(prefix + "_grip_open_button", 6);
    hand.grip_step_deg = this->declare_parameter<double>(prefix + "_grip_step_deg", 2.0);
    hand.grip_min_deg = this->declare_parameter<double>(prefix + "_grip_min_deg", -20.0);
    hand.grip_max_deg = this->declare_parameter<double>(prefix + "_grip_max_deg", 20.0);
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto now = this->get_clock()->now();
    double dt = fallback_dt_;
    if (have_last_joy_time_) {
      dt = (now - last_joy_time_).seconds();
    }
    if (dt <= 0.0) {
      dt = fallback_dt_;
    }
    last_joy_time_ = now;
    have_last_joy_time_ = true;

    update_hand(left_hand_, msg, dt);
    if (right_hand_.enabled) {
      update_hand(right_hand_, msg, dt);
    }
  }

  void update_hand(HandParams & hand, const sensor_msgs::msg::Joy::SharedPtr & msg, double dt)
  {
    if (!hand.enabled || !hand.publisher) {
      return;
    }

    if (hand.wrist_axis >= 0 && static_cast<size_t>(hand.wrist_axis) < msg->axes.size()) {
      double axis = msg->axes[hand.wrist_axis];
      if (std::fabs(axis) < hand.wrist_deadzone) {
        axis = 0.0;
      }
      if (hand.wrist_invert) {
        axis = -axis;
      }
      hand.wrist_deg += axis * hand.wrist_speed_deg_per_sec * dt;
      hand.wrist_deg = clamp(hand.wrist_deg, hand.wrist_min_deg, hand.wrist_max_deg);
    }

    if (hand.grip_close_button >= 0 &&
      static_cast<size_t>(hand.grip_close_button) < msg->buttons.size() &&
      msg->buttons[hand.grip_close_button] == 1)
    {
      hand.grip_deg += hand.grip_step_deg;
    }
    if (hand.grip_open_button >= 0 &&
      static_cast<size_t>(hand.grip_open_button) < msg->buttons.size() &&
      msg->buttons[hand.grip_open_button] == 1)
    {
      hand.grip_deg -= hand.grip_step_deg;
    }
    hand.grip_deg = clamp(hand.grip_deg, hand.grip_min_deg, hand.grip_max_deg);

    std_msgs::msg::Float32MultiArray cmd;
    cmd.data.resize(2);
    // left_hand_dxl_node expects [grip, wrist]
    cmd.data[0] = static_cast<float>(hand.grip_deg);
    cmd.data[1] = static_cast<float>(hand.wrist_deg);
    hand.publisher->publish(cmd);
  }

  std::string joy_topic_;
  double publish_rate_hz_;
  double fallback_dt_;

  HandParams left_hand_;
  HandParams right_hand_;
  bool enable_right_hand_{false};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Time last_joy_time_;
  bool have_last_joy_time_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandsTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
