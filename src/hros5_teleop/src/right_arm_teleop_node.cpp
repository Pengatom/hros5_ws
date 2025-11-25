#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace
{
double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double ticks_to_deg(int ticks)
{
  constexpr double ticks_per_deg = 4095.0 / 360.0;
  return (static_cast<double>(ticks) / ticks_per_deg) - 180.0;
}
}  // namespace

class RightArmTeleopNode : public rclcpp::Node
{
public:
  RightArmTeleopNode()
  : Node("right_arm_teleop_node")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "joy");
    cmd_topic_ = declare_parameter<std::string>("command_topic", "/hros5/right_arm/target_angles_deg");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    fallback_dt_ = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.02;
    echo_via_bridge_ = declare_parameter<bool>("echo_via_bridge", true);
    echo_request_topic_ = declare_parameter<std::string>(
      "echo_request_topic", "/hros5/right_arm/echo_positions");

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

    grip_close_button_ = declare_parameter<int>("grip_close_button", 5);  // R1
    grip_open_button_  = declare_parameter<int>("grip_open_button", 7);   // R2 (digital)
    grip_step_deg_     = declare_parameter<double>("grip_step_deg", 2.0);
    grip_min_deg_      = declare_parameter<double>("grip_min_deg", -50.0);
    grip_max_deg_      = declare_parameter<double>("grip_max_deg", 30.0);

    debug_toggle_button_ = declare_parameter<int>("debug_toggle_button", 1);  // Circle
    echo_position_button_ = declare_parameter<int>("echo_position_button", 0);  // Cross/other

    preset_home_deg_ = declare_parameter<std::vector<double>>(
      "preset_home_deg", {0.0, 0.0, 0.0, 0.0, 0.0});
    preset_ready_deg_ = declare_parameter<std::vector<double>>(
      "preset_ready_deg", {20.0, 20.0, 60.0, 0.0, 0.0});

    joint_min_deg_ = declare_parameter<std::vector<double>>(
      "joint_min_deg", {-178.0, -140.0, -144.0, -130.0, -58.0});
    joint_max_deg_ = declare_parameter<std::vector<double>>(
      "joint_max_deg", {178.0, 140.0, 100.0, 130.0, 32.0});

    if (echo_via_bridge_) {
      echo_pub_ = create_publisher<std_msgs::msg::Empty>(echo_request_topic_, 10);
    } else {
      dxl_device_ = declare_parameter<std::string>("dxl_device", "/dev/dxl");
      dxl_baud_ = declare_parameter<int>("dxl_baud", 1'000'000);
      dxl_protocol_ = declare_parameter<double>("dxl_protocol", 2.0);
      auto servo_ids_param = declare_parameter<std::vector<int64_t>>(
        "servo_ids", {1, 3, 5, 23, 21});
      servo_ids_.assign(servo_ids_param.begin(), servo_ids_param.end());
      present_deg_.assign(servo_ids_.size(), 0.0);

      port_ = dynamixel::PortHandler::getPortHandler(dxl_device_.c_str());
      packet_ = dynamixel::PacketHandler::getPacketHandler(dxl_protocol_);

      if (!port_->openPort()) {
        RCLCPP_ERROR(get_logger(), "Failed to open Dynamixel port: %s", dxl_device_.c_str());
      } else if (!port_->setBaudRate(dxl_baud_)) {
        RCLCPP_ERROR(get_logger(), "Failed to set baud rate to %d", dxl_baud_);
        port_->closePort();
      } else {
        bus_ok_ = true;
        RCLCPP_INFO(get_logger(), "Dynamixel bus ready on %s @ %d", dxl_device_.c_str(), dxl_baud_);
      }
    }

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(cmd_topic_, 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10, std::bind(&RightArmTeleopNode::joy_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&RightArmTeleopNode::publish_command, this));

    RCLCPP_INFO(get_logger(), "RightArmTeleopNode started. Debug mode OFF.");
  }

  ~RightArmTeleopNode() override
  {
    if (!echo_via_bridge_ && bus_ok_ && port_ != nullptr) {
      port_->closePort();
    }
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
    handle_echo_position(msg);
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
      RCLCPP_INFO(get_logger(), "Right arm debug mode %s", debug_enabled_ ? "ON" : "OFF");
    }
    last_toggle_pressed_ = pressed;
  }

  void handle_echo_position(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    bool x_pressed = echo_position_button_ >= 0 &&
      static_cast<size_t>(echo_position_button_) < msg->buttons.size() &&
      msg->buttons[echo_position_button_] == 1;
    if (x_pressed && !last_echo_pressed_) {
      if (echo_via_bridge_) {
        if (echo_pub_) {
          std_msgs::msg::Empty req;
          echo_pub_->publish(req);
        } else {
          RCLCPP_WARN(get_logger(), "Echo publisher not ready");
        }
      } else if (!bus_ok_) {
        RCLCPP_WARN(get_logger(), "Dynamixel bus not ready; cannot read positions.");
      } else {
        read_and_log_positions();
      }
    }
    last_echo_pressed_ = x_pressed;
  }

  void handle_dpad(const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
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
      RCLCPP_INFO(get_logger(), "Right arm step speed: %.1f deg/s", step_deg_per_sec_);
    }
    if (down && !last_dpad_down_) {
      step_deg_per_sec_ = clamp(step_deg_per_sec_ - step_adjust_deg_, step_min_deg_, step_max_deg_);
      RCLCPP_INFO(get_logger(), "Right arm step speed: %.1f deg/s", step_deg_per_sec_);
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

  void read_and_log_positions()
  {
    constexpr uint16_t ADDR_PRESENT_POSITION = 132;
    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary.precision(2);

    std::ostringstream polling;
    polling << "Polling servos: ";
    for (size_t i = 0; i < servo_ids_.size(); ++i) {
      polling << servo_ids_[i];
      if (i + 1 < servo_ids_.size()) {
        polling << ", ";
      }
    }

    char time_buf[9] = {0};
    {
      auto now = std::chrono::system_clock::now();
      std::time_t tt = std::chrono::system_clock::to_time_t(now);
      std::tm tm{};
#ifdef _WIN32
      localtime_s(&tm, &tt);
#else
      localtime_r(&tt, &tm);
#endif
      std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &tm);
    }

    for (size_t i = 0; i < servo_ids_.size(); ++i) {
      uint8_t err = 0;
      uint32_t pos = 0;
      int res = packet_->read4ByteTxRx(
        port_, static_cast<uint8_t>(servo_ids_[i]), ADDR_PRESENT_POSITION, &pos, &err);
      if (res != COMM_SUCCESS) {
        summary << "[id " << servo_ids_[i] << ": comm err " << res << "] ";
        continue;
      }
      if (err) {
        summary << "[id " << servo_ids_[i] << ": packet err " << static_cast<int>(err) << "] ";
        continue;
      }
      int ticks = static_cast<int>(pos);
      present_deg_[i] = ticks_to_deg(ticks);
      summary << "id " << servo_ids_[i] << ": " << present_deg_[i] << " deg; ";

      std::ostringstream detail;
      detail.setf(std::ios::fixed);
      detail << "[" << time_buf << "] ID "
             << std::setw(3) << servo_ids_[i] << ": "
             << std::setw(5) << ticks << " ticks ("
             << std::showpos << std::setprecision(2) << std::setw(7) << present_deg_[i]
             << " deg)";
      RCLCPP_INFO(get_logger(), "%s", detail.str().c_str());
    }

    RCLCPP_INFO(get_logger(), "%s", polling.str().c_str());
    RCLCPP_INFO(get_logger(), "Present positions: %s", summary.str().c_str());
  }

  std::string joy_topic_;
  std::string cmd_topic_;
  double publish_rate_hz_{30.0};
  double fallback_dt_{0.02};
  bool echo_via_bridge_{true};
  std::string echo_request_topic_;

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

  int grip_close_button_{5};
  int grip_open_button_{7};
  double grip_step_deg_{2.0};
  double grip_min_deg_{-50.0};
  double grip_max_deg_{30.0};

  int debug_toggle_button_{1};
  int echo_position_button_{0};

  std::vector<double> preset_home_deg_{0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> preset_ready_deg_{20.0, 20.0, 60.0, 0.0, 0.0};
  std::vector<double> joint_min_deg_{-178.0, -140.0, -144.0, -130.0, -58.0};
  std::vector<double> joint_max_deg_{178.0, 140.0, 100.0, 130.0, 32.0};
  std::array<double, 5> joint_targets_deg_{0.0, 0.0, 0.0, 0.0, 0.0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr echo_pub_;

  std::string dxl_device_;
  int dxl_baud_{1'000'000};
  double dxl_protocol_{2.0};
  std::vector<int> servo_ids_;
  std::vector<double> present_deg_;
  bool bus_ok_{false};
  dynamixel::PortHandler * port_{nullptr};
  dynamixel::PacketHandler * packet_{nullptr};

  rclcpp::Time last_joy_time_;
  bool have_last_joy_time_{false};
  bool debug_enabled_{false};
  bool last_toggle_pressed_{false};
  bool last_echo_pressed_{false};
  bool last_dpad_up_{false}, last_dpad_down_{false}, last_dpad_left_{false}, last_dpad_right_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RightArmTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
