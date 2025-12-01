#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>

namespace {

double clip(double v, double lo, double hi){
  return std::max(lo, std::min(hi, v));
}

double axis_value(const sensor_msgs::msg::Joy::SharedPtr& msg, int axis_index, double deadzone){
  if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg->axes.size()) {
    return 0.0;
  }
  double raw = msg->axes[axis_index];
  if (std::fabs(raw) < deadzone) return 0.0;
  return raw;
}

} // namespace

class RightLegTeleopNode : public rclcpp::Node
{
public:
  RightLegTeleopNode()
  : Node("right_leg_teleop_node")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "joy");
    cmd_topic_ = declare_parameter<std::string>("command_topic", "/hros5/right_leg/target_angles_deg");
    echo_request_topic_ = declare_parameter<std::string>("echo_request_topic", "/hros5/right_leg/echo_positions");
    echo_position_button_ = declare_parameter<int>("echo_position_button", 0);  // PS4 X button
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    deadzone_ = declare_parameter<double>("deadzone", 0.1);
    center_button_ = declare_parameter<int>("center_button", 3);  // PS4: triangle

    hip_yaw_axis_ = declare_parameter<int>("hip_yaw_axis", 6);        // D-pad horizontal
    hip_roll_axis_ = declare_parameter<int>("hip_roll_axis", 0);      // LX
    hip_pitch_axis_ = declare_parameter<int>("hip_pitch_axis", 1);    // LY
    knee_pitch_axis_ = declare_parameter<int>("knee_pitch_axis", 4);  // RY
    ankle_pitch_axis_ = declare_parameter<int>("ankle_pitch_axis", 7);// D-pad vertical
    ankle_roll_axis_ = declare_parameter<int>("ankle_roll_axis", 3);  // RX

    invert_hip_yaw_ = declare_parameter<bool>("invert_hip_yaw", false);
    invert_hip_roll_ = declare_parameter<bool>("invert_hip_roll", false);
    invert_hip_pitch_ = declare_parameter<bool>("invert_hip_pitch", true);
    invert_knee_pitch_ = declare_parameter<bool>("invert_knee_pitch", true);
    invert_ankle_pitch_ = declare_parameter<bool>("invert_ankle_pitch", true);
    invert_ankle_roll_ = declare_parameter<bool>("invert_ankle_roll", false);

    hip_yaw_max_deg_ = declare_parameter<double>("hip_yaw_max_deg", 45.0);
    hip_roll_max_deg_ = declare_parameter<double>("hip_roll_max_deg", 30.0);
    hip_pitch_max_deg_ = declare_parameter<double>("hip_pitch_max_deg", 50.0);
    knee_pitch_max_deg_ = declare_parameter<double>("knee_pitch_max_deg", 70.0);
    ankle_pitch_max_deg_ = declare_parameter<double>("ankle_pitch_max_deg", 45.0);
    ankle_roll_max_deg_ = declare_parameter<double>("ankle_roll_max_deg", 25.0);

    auto forward = declare_parameter<std::vector<double>>(
      "preset_forward_deg", {0.0, 0.0, 15.0, 30.0, -15.0, 0.0});
    auto backward = declare_parameter<std::vector<double>>(
      "preset_backward_deg", {0.0, 0.0, -12.0, 20.0, 10.0, 0.0});
    auto leg_down = declare_parameter<std::vector<double>>(
      "preset_leg_down_deg", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    auto leg_up = declare_parameter<std::vector<double>>(
      "preset_leg_up_deg", {0.0, 0.0, 20.0, 45.0, -20.0, 0.0});

    preset_forward_ = normalize_preset(forward);
    preset_backward_ = normalize_preset(backward);
    preset_leg_down_ = normalize_preset(leg_down);
    preset_leg_up_ = normalize_preset(leg_up);

    std::fill(commands_deg_.begin(), commands_deg_.end(), 0.0);

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(cmd_topic_, 10);
    echo_pub_ = create_publisher<std_msgs::msg::Empty>(echo_request_topic_, 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10, std::bind(&RightLegTeleopNode::joy_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&RightLegTeleopNode::publish_command, this));

    RCLCPP_INFO(get_logger(), "RightLegTeleopNode started.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (apply_presets(msg)) {
      return;
    }

    apply_axis_if_active(msg, hip_yaw_axis_, hip_yaw_max_deg_, invert_hip_yaw_, 0);
    apply_axis_if_active(msg, hip_roll_axis_, hip_roll_max_deg_, invert_hip_roll_, 1);
    apply_axis_if_active(msg, hip_pitch_axis_, hip_pitch_max_deg_, invert_hip_pitch_, 2);
    apply_axis_if_active(msg, knee_pitch_axis_, knee_pitch_max_deg_, invert_knee_pitch_, 3);
    apply_axis_if_active(msg, ankle_pitch_axis_, ankle_pitch_max_deg_, invert_ankle_pitch_, 4);
    apply_axis_if_active(msg, ankle_roll_axis_, ankle_roll_max_deg_, invert_ankle_roll_, 5);

    if (center_button_ >= 0 &&
        static_cast<size_t>(center_button_) < msg->buttons.size() &&
        msg->buttons[center_button_] == 1)
    {
      std::fill(commands_deg_.begin(), commands_deg_.end(), 0.0);
    }

    handle_echo_position(msg);
  }

  double scaled_axis(const sensor_msgs::msg::Joy::SharedPtr& msg, int axis_index,
                     double max_deg, bool invert) const
  {
    double raw = axis_value(msg, axis_index, deadzone_);
    double value = invert ? -raw : raw;
    return clip(value * max_deg, -max_deg, max_deg);
  }

  void apply_axis_if_active(const sensor_msgs::msg::Joy::SharedPtr& msg, int axis_index,
                            double max_deg, bool invert, size_t command_index)
  {
    double raw = axis_value(msg, axis_index, deadzone_);
    if (raw == 0.0) {
      return;
    }
    double value = invert ? -raw : raw;
    commands_deg_[command_index] = clip(value * max_deg, -max_deg, max_deg);
  }

  bool apply_presets(const sensor_msgs::msg::Joy::SharedPtr& msg)
  {
    double v_axis = axis_value(msg, ankle_pitch_axis_, deadzone_);
    double h_axis = axis_value(msg, hip_yaw_axis_, deadzone_);

    bool up = v_axis > 0.5;
    bool down = v_axis < -0.5;
    bool left = h_axis > 0.5;
    bool right = h_axis < -0.5;

    if (up) {
      apply_preset(preset_leg_up_, "leg up");
      return true;
    }
    if (down) {
      apply_preset(preset_leg_down_, "leg down");
      return true;
    }
    if (left) {
      apply_preset(preset_forward_, "forward reach");
      return true;
    }
    if (right) {
      apply_preset(preset_backward_, "backward reach");
      return true;
    }
    return false;
  }

  void apply_preset(const std::array<double, 6>& preset, const char* label)
  {
    commands_deg_ = preset;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Applied right leg preset: %s [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
      label,
      preset[0], preset[1], preset[2], preset[3], preset[4], preset[5]);
  }

  std::array<double, 6> normalize_preset(const std::vector<double>& preset_vec) const
  {
    std::array<double, 6> preset{};
    for (size_t i = 0; i < preset.size(); ++i) {
      preset[i] = i < preset_vec.size() ? preset_vec[i] : 0.0;
    }
    return preset;
  }

  void publish_command()
  {
    std_msgs::msg::Float32MultiArray cmd;
    cmd.data.assign(commands_deg_.begin(), commands_deg_.end());
    pub_->publish(cmd);
  }

  void handle_echo_position(const sensor_msgs::msg::Joy::SharedPtr& msg)
  {
    bool pressed = echo_position_button_ >= 0 &&
      static_cast<size_t>(echo_position_button_) < msg->buttons.size() &&
      msg->buttons[echo_position_button_] == 1;
    if (pressed && !last_echo_pressed_) {
      if (echo_pub_) {
        echo_pub_->publish(std_msgs::msg::Empty{});
      }
    }
    last_echo_pressed_ = pressed;
  }

  std::string joy_topic_;
  std::string cmd_topic_;
  double publish_rate_hz_{30.0};
  double deadzone_{0.1};
  int center_button_{-1};

  int hip_yaw_axis_{-1};
  int hip_roll_axis_{-1};
  int hip_pitch_axis_{-1};
  int knee_pitch_axis_{-1};
  int ankle_pitch_axis_{-1};
  int ankle_roll_axis_{-1};

  bool invert_hip_yaw_{false};
  bool invert_hip_roll_{false};
  bool invert_hip_pitch_{false};
  bool invert_knee_pitch_{false};
  bool invert_ankle_pitch_{false};
  bool invert_ankle_roll_{false};

  double hip_yaw_max_deg_{0.0};
  double hip_roll_max_deg_{0.0};
  double hip_pitch_max_deg_{0.0};
  double knee_pitch_max_deg_{0.0};
  double ankle_pitch_max_deg_{0.0};
  double ankle_roll_max_deg_{0.0};

  std::array<double, 6> preset_forward_{};
  std::array<double, 6> preset_backward_{};
  std::array<double, 6> preset_leg_down_{};
  std::array<double, 6> preset_leg_up_{};

  std::array<double, 6> commands_deg_{};
  std::string echo_request_topic_;
  int echo_position_button_{0};
  bool last_echo_pressed_{false};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr echo_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RightLegTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
