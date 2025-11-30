#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "hros5_dynamixel_bridge/dxl_bus_config.hpp"
#include <chrono>
#include <filesystem>
#include <unordered_map>
#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>

using std::placeholders::_1;
using namespace std::chrono_literals;

constexpr double TICKS_PER_DEG = 4095.0 / 360.0;

constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_PROFILE_ACCEL    = 108;
constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
constexpr uint16_t ADDR_GOAL_POSITION    = 116;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;

namespace {

struct JointInfo {
  int id{-1};
  int cw_limit{-1};
  int ccw_limit{-1};
  double offset_deg{0.0};
};

struct JointState {
  std::string name;
  std::string prefix;
  JointInfo info;
  int id{-1};
  double current_deg{0.0};
  double target_deg{0.0};
  double measured_deg{0.0};
  double last_measured_deg{0.0};
  double stall_time_s{0.0};
  bool have_measured{false};
  bool last_profile_close{false};
  bool last_profile_soft{false};
  bool profile_set{false};
};

int read_limit(const YAML::Node& node){
  if (!node || !node.IsScalar()) return -1;
  auto value = node.as<std::string>();
  if (value == "TBD") return -1;
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    return -1;
  }
}

std::string resolve_config_path(const std::string& config_file,
                                const std::string& config_package){
  std::filesystem::path cfg(config_file);
  if (cfg.is_absolute()) return cfg.string();
  auto share = ament_index_cpp::get_package_share_directory(config_package);
  return (std::filesystem::path(share) / cfg).string();
}

std::unordered_map<std::string, JointInfo> load_joint_info(const std::string& config_path){
  YAML::Node config = YAML::LoadFile(config_path);
  if (!config["joints"]) {
    throw std::runtime_error("joints.yaml missing 'joints' key");
  }
  std::unordered_map<std::string, JointInfo> joints;
  for (const auto& node : config["joints"]) {
    if (!node["name"] || !node["id"]) {
      throw std::runtime_error("joint entry missing name or id");
    }
    JointInfo info;
    info.id = node["id"].as<int>();
    info.cw_limit = read_limit(node["cw_limit"]);
    info.ccw_limit = read_limit(node["ccw_limit"]);
    info.offset_deg = (node["offset_deg"] && node["offset_deg"].IsScalar())
      ? node["offset_deg"].as<double>()
      : 0.0;
    joints[node["name"].as<std::string>()] = info;
  }
  return joints;
}

double ticks_to_deg(int ticks){
  return (static_cast<double>(ticks) / TICKS_PER_DEG) - 180.0;
}

int clamp_ticks(int ticks, const JointInfo& info){
  if (info.cw_limit >= 0 && info.ccw_limit >= 0) {
    return std::max(info.cw_limit, std::min(info.ccw_limit, ticks));
  }
  return std::max(0, std::min(4095, ticks));
}

std::string param_name(const std::string& prefix, const std::string& suffix){
  return prefix + "_" + suffix;
}

} // namespace

class DxlNode : public rclcpp::Node {
public:
  DxlNode() : rclcpp::Node("left_arm_dxl_node") {
    bus_config_ = declare_and_get_bus_config(*this);
    declare_parameter<double>("kp", 0.6);
    declare_parameter<double>("max_step_deg", 2.0);
    declare_parameter<double>("grip_max_step_deg", 3.0);
    declare_parameter<std::string>("config_package", "hros5_dynamixel_bridge");
    declare_parameter<std::string>("config_file", "config/arm_servos.yaml");
    declare_parameter<std::string>("command_topic", "/hros5/left_arm/target_angles_deg");
    declare_parameter<std::string>("echo_request_topic", "/hros5/left_arm/echo_positions");
    declare_parameter<double>("profile_velocity", 80.0);
    declare_parameter<double>("profile_accel", 20.0);
    declare_parameter<double>("grip_profile_velocity", 200.0);  // kept for compatibility
    declare_parameter<double>("grip_profile_accel", 60.0);      // kept for compatibility
    declare_parameter<double>("grip_profile_velocity_close", 140.0);
    declare_parameter<double>("grip_profile_accel_close", 70.0);
    declare_parameter<double>("grip_profile_velocity_open", 220.0);
    declare_parameter<double>("grip_profile_accel_open", 110.0);
    declare_parameter<double>("grip_profile_velocity_close_soft", 70.0);
    declare_parameter<double>("grip_profile_accel_close_soft", 35.0);
    declare_parameter<double>("grip_soft_zone_deg", 6.0);
    declare_parameter<bool>("grip_direct_command", true);
    declare_parameter<double>("grip_target_slew_deg", 5.0);
    declare_parameter<double>("grip_stall_motion_deg", 0.2);
    declare_parameter<double>("grip_stall_timeout_sec", 0.18);
    declare_parameter<double>("grip_release_margin_deg", 1.5);
    declare_parameter<double>("grip_command_gap_deg", 0.7);

    std::string config_path = resolve_config_path(
      get_parameter("config_file").as_string(),
      get_parameter("config_package").as_string());
    auto joints = load_joint_info(config_path);

    const std::array<std::pair<std::string, std::string>, 5> joint_names{{
      {"LShoulderPitch", "lshoulder_pitch"},
      {"LShoulderRoll",  "lshoulder_roll"},
      {"LElbowPitch",    "lelbow_pitch"},
      {"LWrist",         "lwrist"},
      {"LGrip",          "lgrip"},
    }};

    for (const auto& [joint_name, prefix] : joint_names) {
      auto it = joints.find(joint_name);
      if (it == joints.end()) {
        throw std::runtime_error("Missing joint in config: " + joint_name);
      }
      double min_default = it->second.cw_limit >= 0 ? ticks_to_deg(it->second.cw_limit) : -120.0;
      double max_default = it->second.ccw_limit >= 0 ? ticks_to_deg(it->second.ccw_limit) : 120.0;

      declare_parameter<double>(param_name(prefix, "min_deg"), min_default);
      declare_parameter<double>(param_name(prefix, "max_deg"), max_default);
      declare_parameter<double>(param_name(prefix, "offset_deg"), it->second.offset_deg);
      declare_parameter<bool>(param_name("invert", prefix), false);

      JointState state;
      state.name = joint_name;
      state.prefix = prefix;
      state.info = it->second;
      state.id = it->second.id;
      joints_.push_back(state);
    }

    port_ = dynamixel::PortHandler::getPortHandler(bus_config_.device.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(bus_config_.protocol_version);

    if (!port_->openPort()) {
      throw std::runtime_error("openPort failed for " + bus_config_.device);
    }
    if (!port_->setBaudRate(bus_config_.baud)) {
      throw std::runtime_error("setBaudRate failed");
    }
    if (bus_config_.packet_timeout_ms > 0.0) {
      port_->setPacketTimeout(bus_config_.packet_timeout_ms);
    }

    double base_profile_velocity = get_parameter("profile_velocity").as_double();
    double base_profile_accel = get_parameter("profile_accel").as_double();
    double grip_profile_velocity_close = get_parameter("grip_profile_velocity_close").as_double();
    double grip_profile_accel_close = get_parameter("grip_profile_accel_close").as_double();

    for (auto& joint : joints_) {
      write1(joint.id, ADDR_TORQUE_ENABLE, 0);
      write1(joint.id, ADDR_OPERATING_MODE, 3);
      double vel = is_gripper(joint) ? grip_profile_velocity_close : base_profile_velocity;
      double acc = is_gripper(joint) ? grip_profile_accel_close : base_profile_accel;
      write4(joint.id, ADDR_PROFILE_VELOCITY, static_cast<uint32_t>(vel));
      write4(joint.id, ADDR_PROFILE_ACCEL, static_cast<uint32_t>(acc));
      joint.last_profile_close = is_gripper(joint);
      joint.last_profile_soft = false;
      joint.profile_set = true;
    }
    for (const auto& joint : joints_) {
      write1(joint.id, ADDR_TORQUE_ENABLE, 1);
    }

    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      get_parameter("command_topic").as_string(), 10, std::bind(&DxlNode::cb, this, _1));

    echo_sub_ = create_subscription<std_msgs::msg::Empty>(
      get_parameter("echo_request_topic").as_string(), 10,
      std::bind(&DxlNode::handle_echo_request, this, _1));

    timer_ = create_wall_timer(20ms, std::bind(&DxlNode::update, this));
  }

  ~DxlNode() override {
    for (const auto& joint : joints_) {
      write1(joint.id, ADDR_TORQUE_ENABLE, 0);
    }
    port_->closePort();
  }

private:
  static double clip(double x, double lo, double hi){ return std::max(lo, std::min(hi, x)); }

  bool is_gripper(const JointState& joint) const {
    return joint.prefix == "lgrip";
  }

  bool update_grip_measurement(JointState& joint){
    uint32_t pos = 0;
    std::string label = "read pos id=" + std::to_string(joint.id);
    bool ok = dxl_with_retry(
      bus_config_, port_, this->get_logger(), label.c_str(),
      [&](uint8_t& err){
        return packet_->read4ByteTxRx(
          port_, static_cast<uint8_t>(joint.id), ADDR_PRESENT_POSITION, &pos, &err);
      });
    if (!ok) {
      return false;
    }
    double deg = ticks_to_deg(static_cast<int>(pos));
    joint.measured_deg = deg;
    if (!joint.have_measured) {
      joint.last_measured_deg = deg;
      joint.have_measured = true;
      joint.stall_time_s = 0.0;
    }
    return true;
  }

  void apply_grip_profile(JointState& joint, bool closing, bool soft){
    double vel = closing
      ? (soft ? get_parameter("grip_profile_velocity_close_soft").as_double()
              : get_parameter("grip_profile_velocity_close").as_double())
      : get_parameter("grip_profile_velocity_open").as_double();
    double acc = closing
      ? (soft ? get_parameter("grip_profile_accel_close_soft").as_double()
              : get_parameter("grip_profile_accel_close").as_double())
      : get_parameter("grip_profile_accel_open").as_double();

    if (joint.profile_set && joint.last_profile_close == closing && joint.last_profile_soft == soft) {
      return;
    }
    joint.profile_set = true;
    joint.last_profile_close = closing;
    joint.last_profile_soft = soft;
    write4(joint.id, ADDR_PROFILE_VELOCITY, static_cast<uint32_t>(vel));
    write4(joint.id, ADDR_PROFILE_ACCEL, static_cast<uint32_t>(acc));
  }

  double apply_grip_guard(JointState& joint, double desired_target, double dt){
    if (grip_contact_active_ && !joint.have_measured) {
      return std::max(desired_target, grip_contact_deg_);
    }
    if (!joint.have_measured) return desired_target;

    double release_margin = get_parameter("grip_release_margin_deg").as_double();
    if (grip_contact_active_ && desired_target > grip_contact_deg_ + release_margin) {
      grip_contact_active_ = false;
      joint.stall_time_s = 0.0;
    }

    double motion_eps = get_parameter("grip_stall_motion_deg").as_double();
    double stall_timeout = get_parameter("grip_stall_timeout_sec").as_double();
    double command_gap = get_parameter("grip_command_gap_deg").as_double();

    double movement = std::fabs(joint.measured_deg - joint.last_measured_deg);
    if (movement > motion_eps) {
      joint.stall_time_s = 0.0;
    }

    if (grip_contact_active_) {
      joint.last_measured_deg = joint.measured_deg;
      return std::max(desired_target, grip_contact_deg_);
    }

    double closing_gap = joint.measured_deg - desired_target;
    bool requesting_more_close = closing_gap > command_gap;
    if (requesting_more_close && movement < motion_eps) {
      joint.stall_time_s += dt;
    } else {
      joint.stall_time_s = 0.0;
    }

    if (requesting_more_close && joint.stall_time_s >= stall_timeout) {
      grip_contact_active_ = true;
      grip_contact_deg_ = joint.measured_deg;
      joint.last_measured_deg = joint.measured_deg;
      RCLCPP_INFO(this->get_logger(), "Grip contact detected at %.2f deg, holding", grip_contact_deg_);
      return grip_contact_deg_;
    }

    joint.last_measured_deg = joint.measured_deg;
    return desired_target;
  }

  void cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < joints_.size()) return;
    double max_step = get_parameter("max_step_deg").as_double();
    (void)get_parameter("kp"); // retained for compatibility
    (void)max_step;

    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto& joint = joints_[i];
      double min_deg = get_parameter(param_name(joint.prefix, "min_deg")).as_double();
      double max_deg = get_parameter(param_name(joint.prefix, "max_deg")).as_double();
      bool invert = get_parameter(param_name("invert", joint.prefix)).as_bool();
      double cmd = msg->data[i];
      double target = invert ? -cmd : cmd;
      joints_[i].target_deg = clip(target, min_deg, max_deg);
    }
  }

  void update(){
    auto now = this->get_clock()->now();
    double dt = last_update_.nanoseconds() == 0 ? 0.02 : (now - last_update_).seconds();
    last_update_ = now;

    double max_step = get_parameter("max_step_deg").as_double();
    double grip_max_step = get_parameter("grip_max_step_deg").as_double();
    auto step_limited = [max_step, grip_max_step](double step, bool grip){
      double lim = grip ? grip_max_step : max_step;
      return std::max(-lim, std::min(lim, step));
    };
    auto clip_local = [](double x, double lo, double hi){
      return std::max(lo, std::min(hi, x));
    };

    for (auto& joint : joints_) {
      double min_deg = get_parameter(param_name(joint.prefix, "min_deg")).as_double();
      double max_deg = get_parameter(param_name(joint.prefix, "max_deg")).as_double();
      double offset_deg = get_parameter(param_name(joint.prefix, "offset_deg")).as_double();

      if (is_gripper(joint) && update_grip_measurement(joint)) {
        joint.current_deg = joint.measured_deg;
      }

      double desired_target = clip_local(joint.target_deg, min_deg, max_deg);
      if (is_gripper(joint)) {
        desired_target = apply_grip_guard(joint, desired_target, dt);
      }

      bool grip_direct = is_gripper(joint) && get_parameter("grip_direct_command").as_bool();
      if (is_gripper(joint)) {
        bool closing = desired_target < joint.current_deg - 0.05;
        bool opening = desired_target > joint.current_deg + 0.05;
        if (closing || opening) {
          double soft_zone = get_parameter("grip_soft_zone_deg").as_double();
          bool soft = closing && joint.measured_deg <= (min_deg + soft_zone);
          apply_grip_profile(joint, closing, soft);
        }
      }

      if (grip_direct && is_gripper(joint)) {
        double slew = get_parameter("grip_target_slew_deg").as_double();
        double delta = desired_target - joint.current_deg;
        double limited = joint.current_deg + std::max(-slew, std::min(slew, delta));
        joint.current_deg = clip_local(limited, min_deg, max_deg);
      } else {
        double step = step_limited(desired_target - joint.current_deg, is_gripper(joint));
        joint.current_deg = clip_local(joint.current_deg + step, min_deg, max_deg);
      }

      int ticks = static_cast<int>((joint.current_deg + offset_deg + 180.0) * TICKS_PER_DEG);
      ticks = clamp_ticks(ticks, joint.info);
      write4(joint.id, ADDR_GOAL_POSITION, ticks);
    }
  }

  void handle_echo_request(const std_msgs::msg::Empty::SharedPtr) {
    std::ostringstream polling;
    polling << "Polling servos: ";
    for (size_t i = 0; i < joints_.size(); ++i) {
      polling << joints_[i].id;
      if (i + 1 < joints_.size()) {
        polling << ", ";
      }
    }
    RCLCPP_INFO(this->get_logger(), "%s", polling.str().c_str());

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

    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary.precision(2);

    for (const auto& joint : joints_) {
      uint32_t pos = 0;
      std::string label = "echo read id=" + std::to_string(joint.id);
      bool ok = dxl_with_retry(
        bus_config_, port_, this->get_logger(), label.c_str(),
        [&](uint8_t& err){
          return packet_->read4ByteTxRx(
            port_, static_cast<uint8_t>(joint.id), ADDR_PRESENT_POSITION, &pos, &err);
        });
      if (!ok) {
        summary << "[id " << joint.id << ": read failed] ";
        continue;
      }
      int ticks = static_cast<int>(pos);
      double deg = ticks_to_deg(ticks);
      summary << "id " << joint.id << ": " << deg << " deg; ";

      std::ostringstream detail;
      detail.setf(std::ios::fixed);
      detail << "[" << time_buf << "] ID "
             << std::setw(3) << joint.id << ": "
             << std::setw(5) << ticks << " ticks ("
             << std::showpos << std::setprecision(2) << std::setw(7) << deg
             << " deg)";
      RCLCPP_INFO(this->get_logger(), "%s", detail.str().c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Present positions: %s", summary.str().c_str());
  }

  void write1(uint8_t id, uint16_t addr, uint8_t val){
    std::string label = "write1 id=" + std::to_string(id) + " addr=" + std::to_string(addr);
    dxl_with_retry(
      bus_config_, port_, this->get_logger(), label.c_str(),
      [&](uint8_t& err){
        return packet_->write1ByteTxRx(port_, id, addr, val, &err);
      });
  }
  void write4(uint8_t id, uint16_t addr, uint32_t val){
    std::string label = "write4 id=" + std::to_string(id) + " addr=" + std::to_string(addr);
    dxl_with_retry(
      bus_config_, port_, this->get_logger(), label.c_str(),
      [&](uint8_t& err){
        return packet_->write4ByteTxRx(port_, id, addr, val, &err);
      });
  }

  DxlBusConfig bus_config_;
  dynamixel::PortHandler *port_{nullptr};
  dynamixel::PacketHandler *packet_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr echo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<JointState> joints_;
  bool grip_contact_active_{false};
  double grip_contact_deg_{0.0};
  rclcpp::Time last_update_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxlNode>());
  rclcpp::shutdown();
  return 0;
}
