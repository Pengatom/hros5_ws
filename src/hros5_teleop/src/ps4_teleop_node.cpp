// ps4_teleop_node: one PS4 teleop node for the whole robot (#27/#28, docs/STATUS.md §6 step 4).
//
// - One joy_node, one mapping file (config/ps4_mapping.yaml), deadzones.yaml, speed_scales.yaml.
// - Modes (head, left_arm, right_arm, hands, left_leg, right_leg, legs), cycled with Options/Share.
// - Dead-man: nothing moves unless L1 is held. No /joy for joy_timeout_s counts as released.
// - Sticks jog joints in URDF terms; hros5_control/config/joint_directions.yaml converts to
//   servo degrees, so the same stick motion means the same physical motion left and right.
// - Targets are clamped inside the absolute servo limits (hros5_dynamixel_joints_with_limits.yaml).
//   All joints driven by one axis move as a group: if one would pass its limit, the group stops.
// - Publishes the same /hros5/<limb>/target_angles_deg topics and layouts as the per-limb nodes,
//   so it drives the real *_dxl_node executables and hros5_gazebo's sim_dxl_bridge alike.
//   Wrist and grip are sent on the arm topics only (the hand topics would give those servos a
//   second owner on the bus).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace {

constexpr double kTicksPerDeg = 4096.0 / 360.0;

struct Limb {
  std::string name;
  std::vector<std::string> joints;  // message layout of the matching *_dxl_node
};

// Same topics/layouts as hros5_dynamixel_bridge (head, arms, legs).
const std::vector<Limb> kLimbs = {
  {"head", {"HeadYaw", "HeadPitch"}},
  {"left_arm", {"LShoulderPitch", "LShoulderRoll", "LElbowPitch", "LWrist", "LGrip"}},
  {"right_arm", {"RShoulderPitch", "RShoulderRoll", "RElbowPitch", "RWrist", "RGrip"}},
  {"left_leg", {"LHipYaw", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll"}},
  {"right_leg", {"RHipYaw", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"}},
};

struct Binding {
  std::string joint;
  std::string axis;
  double sign{1.0};
};

struct Mode {
  std::string name;
  std::string speed;
  std::string grip;  // optional
  std::vector<Binding> bindings;
};

struct JointLimit {
  double min_deg{-180.0};  // servo degrees, already inside the guard band
  double max_deg{180.0};
};

}  // namespace

class Ps4TeleopNode : public rclcpp::Node {
public:
  Ps4TeleopNode() : Node("ps4_teleop_node") {
    const auto teleop_share = ament_index_cpp::get_package_share_directory("hros5_teleop");
    mapping_file_ = declare_parameter<std::string>("mapping_file", teleop_share + "/config/ps4_mapping.yaml");
    deadzones_file_ = declare_parameter<std::string>("deadzones_file", teleop_share + "/config/deadzones.yaml");
    speeds_file_ = declare_parameter<std::string>("speed_scales_file", teleop_share + "/config/speed_scales.yaml");
    servo_package_ = declare_parameter<std::string>("servo_package", "hros5_control");
    limits_file_ = declare_parameter<std::string>(
      "servo_limits_file", "config/hros5_dynamixel_joints_with_limits.yaml");
    directions_file_ = declare_parameter<std::string>("joint_directions_file", "config/joint_directions.yaml");
    rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    initial_mode_ = declare_parameter<std::string>("initial_mode", "head");

    load_config();

    for (const auto& limb : kLimbs) {
      cmd_pubs_[limb.name] = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/hros5/" + limb.name + "/target_angles_deg", 10);
      echo_pubs_[limb.name] = create_publisher<std_msgs::msg::Empty>(
        "/hros5/" + limb.name + "/echo_positions", 10);
    }
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/hros5/teleop/mode", rclcpp::QoS(1).transient_local());
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg) { on_joy(msg); });

    for (size_t i = 0; i < modes_.size(); ++i) {
      if (modes_[i].name == initial_mode_) mode_index_ = i;
    }
    publish_mode();

    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz_)),
      [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
      "ps4_teleop_node ready: %zu modes, %zu joints. Hold %s to move, %s/%s to change mode. Mode: %s",
      modes_.size(), target_.size(), action_button_name("deadman").c_str(),
      action_button_name("mode_next").c_str(), action_button_name("mode_prev").c_str(),
      modes_[mode_index_].name.c_str());
  }

private:
  // ---------------------------------------------------------------- config
  void load_config() {
    YAML::Node map = YAML::LoadFile(mapping_file_);
    for (const auto& kv : map["axes"]) axes_[kv.first.as<std::string>()] = kv.second.as<int>();
    for (const auto& kv : map["buttons"]) buttons_[kv.first.as<std::string>()] = kv.second.as<int>();
    for (const auto& kv : map["actions"]) {
      const auto button = kv.second.as<std::string>();
      if (!buttons_.count(button)) throw std::runtime_error("action '" + kv.first.as<std::string>() +
                                                             "' uses unknown button '" + button + "'");
      actions_[kv.first.as<std::string>()] = buttons_[button];
    }
    for (const auto& required : {"deadman", "mode_next", "mode_prev"}) {
      if (!actions_.count(required)) throw std::runtime_error(std::string("ps4_mapping.yaml: missing action ") + required);
    }

    YAML::Node dz = YAML::LoadFile(deadzones_file_)["deadzones"];
    stick_deadzone_ = dz["sticks"].as<double>(0.12);
    dpad_deadzone_ = dz["dpad"].as<double>(0.5);
    joy_timeout_s_ = dz["joy_timeout_s"].as<double>(0.5);

    YAML::Node sp = YAML::LoadFile(speeds_file_);
    for (const auto& kv : sp["speed_scales"]) speeds_[kv.first.as<std::string>()] = kv.second.as<double>();
    const double guard = sp["limit_guard_deg"].as<double>(2.0);
    stick_exponent_ = speeds_.count("stick_exponent") ? speeds_["stick_exponent"] : 1.0;

    const auto share = ament_index_cpp::get_package_share_directory(servo_package_);
    for (const auto& j : YAML::LoadFile(share + "/" + limits_file_)["joints"]) {
      const auto name = j["name"].as<std::string>();
      JointLimit lim;
      lim.min_deg = j["cw_limit"].as<double>() / kTicksPerDeg - 180.0 + guard;
      lim.max_deg = j["ccw_limit"].as<double>() / kTicksPerDeg - 180.0 - guard;
      limits_[name] = lim;
    }
    for (const auto& kv : YAML::LoadFile(share + "/" + directions_file_)["direction"]) {
      direction_[kv.first.as<std::string>()] = kv.second.as<double>();
    }

    for (const auto& m : map["modes"]) {
      Mode mode;
      mode.name = m["name"].as<std::string>();
      mode.speed = m["speed"].as<std::string>();
      if (m["grip"]) mode.grip = m["grip"].as<std::string>();
      if (!speeds_.count(mode.speed)) throw std::runtime_error("mode " + mode.name + ": unknown speed " + mode.speed);
      for (const auto& b : m["bindings"]) {
        Binding binding{b["joint"].as<std::string>(), b["axis"].as<std::string>(), b["sign"].as<double>()};
        if (!axes_.count(binding.axis)) throw std::runtime_error("mode " + mode.name + ": unknown axis " + binding.axis);
        check_joint(binding.joint);
        mode.bindings.push_back(binding);
      }
      if (!mode.grip.empty()) check_joint(mode.grip);
      modes_.push_back(mode);
    }
    if (modes_.empty()) throw std::runtime_error("ps4_mapping.yaml: no modes");

    for (const auto& limb : kLimbs) {
      for (const auto& j : limb.joints) {
        check_joint(j);
        // Like the DXL nodes, start every joint at 0 deg (clamped into the limits).
        target_[j] = std::clamp(0.0, limits_[j].min_deg, limits_[j].max_deg);
      }
    }
  }

  void check_joint(const std::string& j) {
    if (!limits_.count(j)) throw std::runtime_error("no servo limits for joint " + j);
    if (!direction_.count(j)) throw std::runtime_error("no direction for joint " + j);
  }

  std::string action_button_name(const std::string& action) const {
    if (!actions_.count(action)) return "?";
    for (const auto& kv : buttons_) if (kv.second == actions_.at(action)) return kv.first;
    return "?";
  }

  // ---------------------------------------------------------------- input
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    joy_ = *msg;
    last_joy_ = now();
    have_joy_ = true;

    // Edge-triggered buttons.
    if (pressed_edge("mode_next")) change_mode(+1);
    if (pressed_edge("mode_prev")) change_mode(-1);
    if (pressed_edge("echo_positions")) {
      for (const auto& limb : limbs_of(modes_[mode_index_])) echo_pubs_[limb]->publish(std_msgs::msg::Empty());
    }
    prev_buttons_ = msg->buttons;
  }

  bool button(const std::string& action) const {
    auto it = actions_.find(action);
    if (it == actions_.end()) return false;
    const int idx = it->second;
    return idx >= 0 && static_cast<size_t>(idx) < joy_.buttons.size() && joy_.buttons[idx] != 0;
  }

  bool pressed_edge(const std::string& action) const {
    auto it = actions_.find(action);
    if (it == actions_.end()) return false;
    const auto idx = static_cast<size_t>(it->second);
    const bool now_down = idx < joy_.buttons.size() && joy_.buttons[idx] != 0;
    const bool was_down = idx < prev_buttons_.size() && prev_buttons_[idx] != 0;
    return now_down && !was_down;
  }

  double axis(const std::string& name) const {
    const int idx = axes_.at(name);
    if (idx < 0 || static_cast<size_t>(idx) >= joy_.axes.size()) return 0.0;
    const double v = joy_.axes[idx];
    const bool dpad = name.rfind("dpad", 0) == 0;
    const double dz = dpad ? dpad_deadzone_ : stick_deadzone_;
    if (std::fabs(v) < dz) return 0.0;
    if (dpad) return v > 0 ? 1.0 : -1.0;
    const double scaled = (std::fabs(v) - dz) / (1.0 - dz);
    return std::copysign(std::pow(std::min(1.0, scaled), stick_exponent_), v);
  }

  void change_mode(int step) {
    const int n = static_cast<int>(modes_.size());
    mode_index_ = static_cast<size_t>((static_cast<int>(mode_index_) + step + n) % n);
    publish_mode();
    RCLCPP_INFO(get_logger(), "Mode: %s", modes_[mode_index_].name.c_str());
  }

  void publish_mode() {
    std_msgs::msg::String m;
    m.data = modes_[mode_index_].name;
    mode_pub_->publish(m);
  }

  std::set<std::string> limbs_of(const Mode& mode) const {
    std::set<std::string> joints;
    for (const auto& b : mode.bindings) joints.insert(b.joint);
    if (!mode.grip.empty()) joints.insert(mode.grip);
    std::set<std::string> limbs;
    for (const auto& limb : kLimbs) {
      for (const auto& j : limb.joints) if (joints.count(j)) limbs.insert(limb.name);
    }
    return limbs;
  }

  // ---------------------------------------------------------------- motion
  // Apply servo-degree deltas as a group: scale all of them down so none leaves its limits.
  void apply_group(const std::map<std::string, double>& delta) {
    double scale = 1.0;
    for (const auto& [j, d] : delta) {
      const double t = target_[j];
      if (d > 0 && t + d > limits_[j].max_deg) scale = std::min(scale, std::max(0.0, (limits_[j].max_deg - t) / d));
      if (d < 0 && t + d < limits_[j].min_deg) scale = std::min(scale, std::max(0.0, (limits_[j].min_deg - t) / d));
    }
    for (const auto& [j, d] : delta) {
      target_[j] = std::clamp(target_[j] + d * scale, limits_[j].min_deg, limits_[j].max_deg);
    }
  }

  void tick() {
    const auto t = now();
    const double dt = std::clamp((t - last_tick_).seconds(), 0.0, 0.1);
    last_tick_ = t;

    const bool joy_alive = have_joy_ && (t - last_joy_).seconds() < joy_timeout_s_;
    if (have_joy_ && !joy_alive && !timeout_warned_) {
      RCLCPP_WARN(get_logger(), "No /joy for %.1f s: holding position (dead-man released).", joy_timeout_s_);
      timeout_warned_ = true;
    }
    if (joy_alive) timeout_warned_ = false;

    const bool deadman = joy_alive && button("deadman");
    if (deadman != last_deadman_) {
      RCLCPP_INFO(get_logger(), deadman ? "Dead-man held: %s active" : "Dead-man released: holding (%s)",
                  modes_[mode_index_].name.c_str());
      last_deadman_ = deadman;
    }

    if (deadman) {
      const Mode& mode = modes_[mode_index_];
      const double speed = speeds_[mode.speed];

      if (button("home")) {
        // Move every joint of this mode towards 0 deg (URDF zero = servo 0 deg).
        std::set<std::string> joints;
        for (const auto& b : mode.bindings) joints.insert(b.joint);
        if (!mode.grip.empty()) joints.insert(mode.grip);
        const double step = speeds_["home"] * dt;
        for (const auto& j : joints) {
          const double goal = std::clamp(0.0, limits_[j].min_deg, limits_[j].max_deg);
          target_[j] += std::clamp(goal - target_[j], -step, step);
        }
      } else {
        // Group the bindings by axis so coupled joints (e.g. a squat) stop together at a limit.
        std::map<std::string, std::map<std::string, double>> groups;
        for (const auto& b : mode.bindings) {
          const double v = axis(b.axis);
          if (v == 0.0) continue;
          const double urdf_delta = b.sign * v * speed * dt;
          groups[b.axis][b.joint] += direction_[b.joint] * urdf_delta;
        }
        for (const auto& [ax, delta] : groups) apply_group(delta);

        if (!mode.grip.empty()) {
          const double g = (button("grip_close") ? 1.0 : 0.0) - (button("grip_open") ? 1.0 : 0.0);
          if (g != 0.0) apply_group({{mode.grip, direction_[mode.grip] * g * speeds_["grip"] * dt}});
        }
      }
    }

    for (const auto& limb : kLimbs) {
      std_msgs::msg::Float32MultiArray msg;
      for (const auto& j : limb.joints) msg.data.push_back(static_cast<float>(target_[j]));
      cmd_pubs_[limb.name]->publish(msg);
    }
  }

  // ---------------------------------------------------------------- state
  std::string mapping_file_, deadzones_file_, speeds_file_, servo_package_, limits_file_, directions_file_;
  std::string initial_mode_;
  double rate_hz_{50.0};
  double stick_deadzone_{0.12}, dpad_deadzone_{0.5}, joy_timeout_s_{0.5}, stick_exponent_{1.0};

  std::map<std::string, int> axes_, buttons_, actions_;
  std::map<std::string, double> speeds_, direction_;
  std::map<std::string, JointLimit> limits_;
  std::vector<Mode> modes_;
  size_t mode_index_{0};

  std::map<std::string, double> target_;  // servo degrees, like the DXL node topics

  sensor_msgs::msg::Joy joy_;
  std::vector<int> prev_buttons_;
  rclcpp::Time last_joy_{0, 0, RCL_ROS_TIME}, last_tick_{0, 0, RCL_ROS_TIME};
  bool have_joy_{false}, timeout_warned_{false}, last_deadman_{false};

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> cmd_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> echo_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Ps4TeleopNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ps4_teleop_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
