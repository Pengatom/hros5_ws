#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <unordered_map>
#include <array>
#include <algorithm>

using std::placeholders::_1;
using namespace std::chrono_literals;

constexpr double TICKS_PER_DEG = 4095.0 / 360.0;

constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_PROFILE_ACCEL    = 108;
constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
constexpr uint16_t ADDR_GOAL_POSITION    = 116;

namespace {

struct JointInfo {
  int id{-1};
  int cw_limit{-1};
  int ccw_limit{-1};
};

struct JointState {
  std::string name;
  std::string prefix;
  JointInfo info;
  int id{-1};
  double current_deg{0.0};
  double target_deg{0.0};
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
  DxlNode() : rclcpp::Node("right_arm_dxl_node") {
    declare_parameter<std::string>("device", "/dev/dxl");
    declare_parameter<int>("baud", 1000000);
    declare_parameter<double>("kp", 0.6);
    declare_parameter<double>("max_step_deg", 2.0);
    declare_parameter<std::string>("config_package", "hros5_control");
    declare_parameter<std::string>("config_file", "config/hros5_dynamixel_joints_with_limits.yaml");
    declare_parameter<std::string>("command_topic", "/hros5/right_arm/target_angles_deg");

    std::string config_path = resolve_config_path(
      get_parameter("config_file").as_string(),
      get_parameter("config_package").as_string());
    auto joints = load_joint_info(config_path);

    const std::array<std::pair<std::string, std::string>, 5> joint_names{{
      {"RShoulderPitch", "rshoulder_pitch"},
      {"RShoulderRoll",  "rshoulder_roll"},
      {"RElbowPitch",    "relbow_pitch"},
      {"RWrist",         "rwrist"},
      {"RGrip",          "rgrip"},
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
      declare_parameter<double>(param_name(prefix, "offset_deg"), 0.0);
      declare_parameter<bool>(param_name("invert", prefix), false);

      JointState state;
      state.name = joint_name;
      state.prefix = prefix;
      state.info = it->second;
      state.id = it->second.id;
      joints_.push_back(state);
    }

    std::string dev = get_parameter("device").as_string();
    int baud = get_parameter("baud").as_int();

    port_ = dynamixel::PortHandler::getPortHandler(dev.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!port_->openPort()) throw std::runtime_error("openPort failed");
    if (!port_->setBaudRate(baud)) throw std::runtime_error("setBaudRate failed");

    for (const auto& joint : joints_) {
      write1(joint.id, ADDR_TORQUE_ENABLE, 0);
      write1(joint.id, ADDR_OPERATING_MODE, 3);
      write4(joint.id, ADDR_PROFILE_VELOCITY, 80);
      write4(joint.id, ADDR_PROFILE_ACCEL, 20);
    }
    for (const auto& joint : joints_) {
      write1(joint.id, ADDR_TORQUE_ENABLE, 1);
    }

    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      get_parameter("command_topic").as_string(), 10, std::bind(&DxlNode::cb, this, _1));

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
    double max_step = get_parameter("max_step_deg").as_double();
    auto step_limited = [max_step](double step){
      return std::max(-max_step, std::min(max_step, step));
    };
    auto clip_local = [](double x, double lo, double hi){
      return std::max(lo, std::min(hi, x));
    };

    for (auto& joint : joints_) {
      double min_deg = get_parameter(param_name(joint.prefix, "min_deg")).as_double();
      double max_deg = get_parameter(param_name(joint.prefix, "max_deg")).as_double();
      double offset_deg = get_parameter(param_name(joint.prefix, "offset_deg")).as_double();

      double step = step_limited(joint.target_deg - joint.current_deg);
      joint.current_deg = clip_local(joint.current_deg + step, min_deg, max_deg);

      int ticks = static_cast<int>((joint.current_deg + offset_deg + 180.0) * TICKS_PER_DEG);
      ticks = clamp_ticks(ticks, joint.info);
      write4(joint.id, ADDR_GOAL_POSITION, ticks);
    }
  }

  void write1(uint8_t id, uint16_t addr, uint8_t val){
    uint8_t err=0; int res = packet_->write1ByteTxRx(port_, id, addr, val, &err);
    if (res != COMM_SUCCESS || err) RCLCPP_WARN(this->get_logger(),
      "write1 id=%d addr=%d err=%d res=%d", id, addr, err, res);
  }
  void write4(uint8_t id, uint16_t addr, uint32_t val){
    uint8_t err=0; int res = packet_->write4ByteTxRx(port_, id, addr, val, &err);
    if (res != COMM_SUCCESS || err) RCLCPP_WARN(this->get_logger(),
      "write4 id=%d addr=%d err=%d res=%d", id, addr, err, res);
  }

  dynamixel::PortHandler *port_{nullptr};
  dynamixel::PacketHandler *packet_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<JointState> joints_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxlNode>());
  rclcpp::shutdown();
  return 0;
}
