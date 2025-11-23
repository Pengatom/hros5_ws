#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <unordered_map>

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

std::string resolve_config_path(const std::string& config_file){
  std::filesystem::path cfg(config_file);
  if (cfg.is_absolute()) return cfg.string();
  auto share = ament_index_cpp::get_package_share_directory("hros5_dynamixel_bridge");
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

} // namespace

class DxlNode : public rclcpp::Node {
public:
  DxlNode() : rclcpp::Node("left_hand_dxl_node") {
    declare_parameter<std::string>("device", "/dev/dxl");
    declare_parameter<int>("baud", 1000000);
    declare_parameter<double>("kp", 0.6);
    declare_parameter<double>("max_step_deg", 2.0);
    declare_parameter<std::string>("config_file", "config/joints.yaml");

    std::string config_path = resolve_config_path(get_parameter("config_file").as_string());
    auto joints = load_joint_info(config_path);
    auto lgrip_it = joints.find("LGrip");
    auto lwrist_it = joints.find("LWrist");
    if (lgrip_it == joints.end() || lwrist_it == joints.end()) {
      throw std::runtime_error("LGrip or LWrist joints missing from joints.yaml");
    }
    lgrip_info_ = lgrip_it->second;
    lwrist_info_ = lwrist_it->second;
    lgrip_id_ = lgrip_info_.id;
    lwrist_id_ = lwrist_info_.id;

    double lgrip_min_default = lgrip_it->second.cw_limit >= 0 ? ticks_to_deg(lgrip_it->second.cw_limit) : -120.0;
    double lgrip_max_default = lgrip_it->second.ccw_limit >= 0 ? ticks_to_deg(lgrip_it->second.ccw_limit) : 120.0;
    double lwrist_min_default = lwrist_it->second.cw_limit >= 0 ? ticks_to_deg(lwrist_it->second.cw_limit) : -30.0;
    double lwrist_max_default = lwrist_it->second.ccw_limit >= 0 ? ticks_to_deg(lwrist_it->second.ccw_limit) : 60.0;

    declare_parameter<double>("lgrip_min_deg", lgrip_min_default);
    declare_parameter<double>("lgrip_max_deg", lgrip_max_default);
    declare_parameter<double>("lwrist_min_deg", lwrist_min_default);
    declare_parameter<double>("lwrist_max_deg", lwrist_max_default);
    declare_parameter<double>("lgrip_offset_deg", 0.0);
    declare_parameter<double>("lwrist_offset_deg", 0.0);
    declare_parameter<bool>("invert_lgrip", false);
    declare_parameter<bool>("invert_lwrist", false);
    declare_parameter<std::string>("error_topic", "/hros5/left_hand/target_angles_deg");

    std::string dev = get_parameter("device").as_string();
    int baud = get_parameter("baud").as_int();

    port_ = dynamixel::PortHandler::getPortHandler(dev.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!port_->openPort()) throw std::runtime_error("openPort failed");
    if (!port_->setBaudRate(baud)) throw std::runtime_error("setBaudRate failed");

    write1(lgrip_id_,  ADDR_TORQUE_ENABLE, 0);
    write1(lwrist_id_, ADDR_TORQUE_ENABLE, 0);
    write1(lgrip_id_,  ADDR_OPERATING_MODE, 3);
    write1(lwrist_id_, ADDR_OPERATING_MODE, 3);
    write4(lgrip_id_,  ADDR_PROFILE_VELOCITY, 80);
    write4(lwrist_id_, ADDR_PROFILE_VELOCITY, 80);
    write4(lgrip_id_,  ADDR_PROFILE_ACCEL,    20);
    write4(lwrist_id_, ADDR_PROFILE_ACCEL,    20);
    write1(lgrip_id_,  ADDR_TORQUE_ENABLE, 1);
    write1(lwrist_id_, ADDR_TORQUE_ENABLE, 1);

    std::string topic = get_parameter("error_topic").as_string();
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      topic, 10, std::bind(&DxlNode::cb, this, _1));

    timer_ = create_wall_timer(20ms, std::bind(&DxlNode::update, this));
  }

  ~DxlNode() override {
    write1(lgrip_id_,  ADDR_TORQUE_ENABLE, 0);
    write1(lwrist_id_, ADDR_TORQUE_ENABLE, 0);
    port_->closePort();
  }

private:
  static double clip(double x, double lo, double hi){ return std::max(lo, std::min(hi, x)); }

  void cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    double lgrip_min  = get_parameter("lgrip_min_deg").as_double();
    double lgrip_max  = get_parameter("lgrip_max_deg").as_double();
    double lwrist_min = get_parameter("lwrist_min_deg").as_double();
    double lwrist_max = get_parameter("lwrist_max_deg").as_double();
    bool invert_lgrip = get_parameter("invert_lgrip").as_bool();
    bool invert_lwrist = get_parameter("invert_lwrist").as_bool();
    double max_step = get_parameter("max_step_deg").as_double();
    (void)get_parameter("kp"); // parameter retained for compatibility, no longer used

    if (msg->data.size() < 2) return;
    double lgrip_cmd  = msg->data[0];  // absolute target LGrip (deg)
    double lwrist_cmd = msg->data[1];  // absolute target LWrist (deg)

    (void)max_step; // consumed in update()
    double lgrip_target  = invert_lgrip  ? -lgrip_cmd  : lgrip_cmd;
    double lwrist_target = invert_lwrist ? -lwrist_cmd : lwrist_cmd;
    target_lgrip_deg_  = clip(lgrip_target,  lgrip_min,  lgrip_max);
    target_lwrist_deg_ = clip(lwrist_target, lwrist_min, lwrist_max);
  }

  void update(){
    double lgrip_off  = get_parameter("lgrip_offset_deg").as_double();
    double lwrist_off = get_parameter("lwrist_offset_deg").as_double();
    double max_step = get_parameter("max_step_deg").as_double();
    double lgrip_min  = get_parameter("lgrip_min_deg").as_double();
    double lgrip_max  = get_parameter("lgrip_max_deg").as_double();
    double lwrist_min = get_parameter("lwrist_min_deg").as_double();
    double lwrist_max = get_parameter("lwrist_max_deg").as_double();

    auto step_limited = [max_step](double step){
      return std::max(-max_step, std::min(max_step, step));
    };
    auto clip = [](double x, double lo, double hi){
      return std::max(lo, std::min(hi, x));
    };

    double lgrip_step  = step_limited(target_lgrip_deg_  - lgrip_deg_);
    double lwrist_step = step_limited(target_lwrist_deg_ - lwrist_deg_);

    lgrip_deg_  = clip(lgrip_deg_  + lgrip_step,  lgrip_min,  lgrip_max);
    lwrist_deg_ = clip(lwrist_deg_ + lwrist_step, lwrist_min, lwrist_max);

    int lgrip_ticks  = static_cast<int>((lgrip_deg_  + lgrip_off + 180.0) * TICKS_PER_DEG);
    int lwrist_ticks = static_cast<int>((lwrist_deg_ + lwrist_off + 180.0) * TICKS_PER_DEG);
    lgrip_ticks = clamp_ticks(lgrip_ticks, lgrip_info_);
    lwrist_ticks = clamp_ticks(lwrist_ticks, lwrist_info_);

    write4(lgrip_id_,  ADDR_GOAL_POSITION, lgrip_ticks);
    write4(lwrist_id_, ADDR_GOAL_POSITION, lwrist_ticks);
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
  JointInfo lgrip_info_;
  JointInfo lwrist_info_;
  int lgrip_id_{-1};
  int lwrist_id_{-1};
  double lgrip_deg_{0.0}, lwrist_deg_{0.0};
  double target_lgrip_deg_{0.0}, target_lwrist_deg_{0.0};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxlNode>());
  rclcpp::shutdown();
  return 0;
}
