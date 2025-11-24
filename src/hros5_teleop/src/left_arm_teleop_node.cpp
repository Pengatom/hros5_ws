#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace
{
double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}
}  // namespace

class LeftArmTeleopNode : public rclcpp::Node
{
public:
  LeftArmTeleopNode()
  : Node("left_arm_teleop_node")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "joy");
    cmd_topic_ = declare_parameter<std::string>("command_topic", "/hros5/left_arm/target_angles_deg");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    fallback_dt_ = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.02;

    shoulder_pitch_axis_ = declare_parameter<int>("shoulder_pitch_axis", 1);  // LY
    shoulder_roll_axis_  = declare_parameter<int>("shoulder_roll_axis", 0);   // LX
    elbow_pitch_axis_    = declare_parameter<int>("elbow_pitch_axis", 4);     // RY
    wrist_axis_          = declare_parameter<int>("wrist_axis", 3);           // RX
    dpad_horizontal_axis_= declare_parameter<int>("dpad_horizontal_axis", 6);
    dpad_vertical_axis_  = declare_parameter<int>("dpad_vertical_axis", 7);
    invert_shoulder_pitch_= declare_parameter<bool>("invert_shoulder_pitch", true);
    invert_shoulder_roll_ = declare_parameter<bool>("invert_shoulder_roll", false);
    invert_elbow_pitch_   = declare_parameter<bool>("invert_elbow_pitch", true);
    invert_wrist_         = declare_parameter<bool>("invert_wrist", false);
    deadzone_             = declare_parameter<double>("deadzone", 0.1);

    step_deg_per_sec_ = declare_parameter<double>("step_deg_per_sec", 60.0);
    step_adjust_deg_  = declare_parameter<double>("step_adjust_deg", 5.0);
    step_min_deg_     = declare_parameter<double>("step_min_deg", 5.0);
    step_max_deg_     = declare_parameter<double>("step_max_deg", 180.0);

    grip_close_button_ = declare_parameter<int>("grip_close_button", 4);  // L1
    grip_open_button_  = declare_parameter<int>("grip_open_button", 6);   // L2 (digital)
    grip_step_deg_     = declare_parameter<double>("grip_step_deg", 2.0);
    grip_min_deg_      = declare_parameter<double>("grip_min_deg", -50.0);
    grip_max_deg_      = declare_parameter<double>("grip_max_deg", 30.0);

    debug_toggle_button_ = declare_parameter<int>("debug_toggle_button", 3);  // Triangle

    preset_home_deg_ = declare_parameter<std::vector<double>>(
      "preset_home_deg", {0.0, 0.0, 0.0, 0.0, 0.0});
    preset_ready_deg_ = declare_parameter<std::vector<double>>(
      "preset_ready_deg", {20.0, -20.0, 60.0, 0.0, 0.0});

    joint_min_deg_ = declare_parameter<std::vector<double>>(
      "joint_min_deg", {-178.0, -140.0, -144.0, -130.0, -58.0});
    joint_max_deg_ = declare_parameter<std::vector<double>>(
      "joint_max_deg", {178.0, 100.0, 100.0, 130.0, 32.0});

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(cmd_topic_, 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10, std::bind(&LeftArmTeleopNode::joy_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&LeftArmTeleopNode::publish_command, this));

    RCLCPP_INFO(get_logger(), "LeftArmTeleopNode started. Debug mode OFF.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto now = get_clock()->now();
    double dt = fallback_dt_;
    if (have_last_joy_time_) {
      dt = (now - last_joy_time_).seconds();
      if (dt <= 0.0) {
        dt = fallback_dt_;
      }
    }
    last_joy_time_ = now;
    have_last_joy_time_ = true;

    handle_debug_toggle(msg);
    handle_dpad(msg);
    handle_gripper(msg);

    if (debug_enabled_) {
      handle_axes(msg, dt);
    }
  }

  void handle_debug_toggle(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    bool pressed = debug_toggle_button_ >= 0 &&
      static_cast<size_t>(debug_toggle_button_) < msg->buttons.size() &&
      msg->buttons[debug_toggle_button_] == 1;
    if (pressed && !last_toggle_pressed_) {
      debug_enabled_ = !debug_enabled_;
      RCLCPP_INFO(get_logger(), "Left arm debug mode %s", debug_enabled_ ? "ON" : "OFF");
    }
    last_toggle_pressed_ = pressed;
  }

  void handle_dpad(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    // Step adjust on up/down edges
    double v_axis = dpad_vertical_axis_ >= 0 &&
      static_cast<size_t>(dpad_vertical_axis_) < msg->axes.size()
        ? msg->axes[dpad_vertical_axis_] : 0.0;
    double h_axis = dpad_horizontal_axis_ >= 0 &&
      static_cast<size_t>(dpad_horizontal_axis_) < msg->axes.size()
        ? msg->axes[dpad_horizontal_axis_] : 0.0;

    bool up = v_axis > 0.5;
    bool down = v_axis < -0.5;
    bool left = h_axis > 0.5;
    bool right = h_axis < -0.5;

    if (up && !last_dpad_up_) {
      step_deg_per_sec_ = clamp(step_deg_per_sec_ + step_adjust_deg_, step_min_deg_, step_max_deg_);
      RCLCPP_INFO(get_logger(), "Left arm step speed: %.1f deg/s", step_deg_per_sec_);
    }
    if (down && !last_dpad_down_) {
      step_deg_per_sec_ = clamp(step_deg_per_sec_ - step_adjust_deg_, step_min_deg_, step_max_deg_);
      RCLCPP_INFO(get_logger(), "Left arm step speed: %.1f deg/s", step_deg_per_sec_);
    }
    if (left && !last_dpad_left_) {
      apply_preset(preset_home_deg_);
    }
    if (right && !last_dpad_right_) {
      apply_preset(preset_ready_deg_);
    }

    last_dpad_up_ = up;
    last_dpad_down_ = down;
    last_dpad_left_ = left;
    last_dpad_right_ = right;
  }

  void apply_preset(const std::vector<double> & preset)
  {
    if (preset.size() != joint_targets_deg_.size()) {
      RCLCPP_WARN(get_logger(), "Preset size mismatch; expected %zu got %zu",
        joint_targets_deg_.size(), preset.size());
      return;
    }
    for (size_t i = 0; i < preset.size(); ++i) {
      joint_targets_deg_[i] = clamp(preset[i], joint_min_deg_[i], joint_max_deg_[i]);
    }
    RCLCPP_INFO(get_logger(), "Applied preset pose");
  }

  void handle_gripper(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    if (grip_close_button_ >= 0 &&
      static_cast<size_t>(grip_close_button_) < msg->buttons.size() &&
      msg->buttons[grip_close_button_] == 1)
    {
      joint_targets_deg_[4] += grip_step_deg_;
    }
    if (grip_open_button_ >= 0 &&
      static_cast<size_t>(grip_open_button_) < msg->buttons.size() &&
      msg->buttons[grip_open_button_] == 1)
    {
      joint_targets_deg_[4] -= grip_step_deg_;
    }
    joint_targets_deg_[4] = clamp(joint_targets_deg_[4], joint_min_deg_[4], joint_max_deg_[4]);
  }

  void handle_axes(const sensor_msgs::msg::Joy::SharedPtr & msg, double dt)
  {
    const std::array<int, 4> axes{shoulder_pitch_axis_, shoulder_roll_axis_, elbow_pitch_axis_,
                                  wrist_axis_};
    const std::array<bool, 4> invert{invert_shoulder_pitch_, invert_shoulder_roll_,
                                     invert_elbow_pitch_, invert_wrist_};

    for (size_t i = 0; i < axes.size(); ++i) {
      int axis_idx = axes[i];
      if (axis_idx < 0 || static_cast<size_t>(axis_idx) >= msg->axes.size()) {
        continue;
      }
      double val = msg->axes[axis_idx];
      if (std::fabs(val) < deadzone_) {
        continue;
      }
      if (invert[i]) {
        val = -val;
      }
      joint_targets_deg_[i] += val * step_deg_per_sec_ * dt;
      joint_targets_deg_[i] = clamp(joint_targets_deg_[i], joint_min_deg_[i], joint_max_deg_[i]);
    }
  }

  void publish_command()
  {
    std_msgs::msg::Float32MultiArray cmd;
    cmd.data.resize(5);
    for (size_t i = 0; i < joint_targets_deg_.size(); ++i) {
      cmd.data[i] = static_cast<float>(joint_targets_deg_[i]);
    }
    pub_->publish(cmd);
  }

  std::string joy_topic_;
  std::string cmd_topic_;
  double publish_rate_hz_{30.0};
  double fallback_dt_{0.02};

  int shoulder_pitch_axis_{1};
  int shoulder_roll_axis_{0};
  int elbow_pitch_axis_{4};
  int wrist_axis_{3};
  int dpad_horizontal_axis_{6};
  int dpad_vertical_axis_{7};
  bool invert_shoulder_pitch_{true};
  bool invert_shoulder_roll_{false};
  bool invert_elbow_pitch_{true};
  bool invert_wrist_{false};
  double deadzone_{0.1};

  double step_deg_per_sec_{60.0};
  double step_adjust_deg_{5.0};
  double step_min_deg_{5.0};
  double step_max_deg_{180.0};

  int grip_close_button_{4};
  int grip_open_button_{6};
  double grip_step_deg_{2.0};
  double grip_min_deg_{-50.0};
  double grip_max_deg_{30.0};

  int debug_toggle_button_{3};

  std::vector<double> preset_home_deg_{0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> preset_ready_deg_{20.0, -20.0, 60.0, 0.0, 0.0};
  std::vector<double> joint_min_deg_{-178.0, -140.0, -144.0, -130.0, -58.0};
  std::vector<double> joint_max_deg_{178.0, 100.0, 100.0, 130.0, 32.0};
  std::array<double, 5> joint_targets_deg_{0.0, 0.0, 0.0, 0.0, 0.0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_joy_time_;
  bool have_last_joy_time_{false};
  bool debug_enabled_{false};
  bool last_toggle_pressed_{false};
  bool last_dpad_up_{false}, last_dpad_down_{false}, last_dpad_left_{false}, last_dpad_right_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LeftArmTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
