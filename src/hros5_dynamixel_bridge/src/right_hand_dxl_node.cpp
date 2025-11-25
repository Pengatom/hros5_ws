#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
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
  DxlNode() : rclcpp::Node("right_hand_dxl_node") {
    declare_parameter<std::string>("device", "/dev/dxl");
    declare_parameter<int>("baud", 1000000);
    declare_parameter<double>("kp", 0.6);
    declare_parameter<double>("max_step_deg", 2.0);
    declare_parameter<std::string>("config_file", "config/joints.yaml");

    std::string config_path = resolve_config_path(get_parameter("config_file").as_string());
    auto joints = load_joint_info(config_path);
    auto rgrip_it = joints.find("RGrip");
    auto rwrist_it = joints.find("RWrist");
    if (rgrip_it == joints.end() || rwrist_it == joints.end()) {
      throw std::runtime_error("RGrip or RWrist joints missing from joints.yaml");
    }
    rgrip_info_ = rgrip_it->second;
    rwrist_info_ = rwrist_it->second;
    rgrip_id_ = rgrip_info_.id;
    rwrist_id_ = rwrist_info_.id;

    double rgrip_min_default = rgrip_it->second.cw_limit >= 0 ? ticks_to_deg(rgrip_it->second.cw_limit) : -120.0;
    double rgrip_max_default = rgrip_it->second.ccw_limit >= 0 ? ticks_to_deg(rgrip_it->second.ccw_limit) : 120.0;
    double rwrist_min_default = rwrist_it->second.cw_limit >= 0 ? ticks_to_deg(rwrist_it->second.cw_limit) : -30.0;
    double rwrist_max_default = rwrist_it->second.ccw_limit >= 0 ? ticks_to_deg(rwrist_it->second.ccw_limit) : 60.0;

    declare_parameter<double>("rgrip_min_deg", rgrip_min_default);
    declare_parameter<double>("rgrip_max_deg", rgrip_max_default);
    declare_parameter<double>("rwrist_min_deg", rwrist_min_default);
    declare_parameter<double>("rwrist_max_deg", rwrist_max_default);
    declare_parameter<double>("rgrip_offset_deg", 0.0);
    declare_parameter<double>("rwrist_offset_deg", 0.0);
    declare_parameter<bool>("invert_rgrip", true);
    declare_parameter<bool>("invert_rwrist", false);
    declare_parameter<std::string>("error_topic", "/hros5/right_hand/target_angles_deg");

    std::string dev = get_parameter("device").as_string();
    int baud = get_parameter("baud").as_int();

    port_ = dynamixel::PortHandler::getPortHandler(dev.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!port_->openPort()) throw std::runtime_error("openPort failed");
    if (!port_->setBaudRate(baud)) throw std::runtime_error("setBaudRate failed");

    write1(rgrip_id_,  ADDR_TORQUE_ENABLE, 0);
    write1(rwrist_id_, ADDR_TORQUE_ENABLE, 0);
    write1(rgrip_id_,  ADDR_OPERATING_MODE, 3);
    write1(rwrist_id_, ADDR_OPERATING_MODE, 3);
    write4(rgrip_id_,  ADDR_PROFILE_VELOCITY, 80);
    write4(rwrist_id_, ADDR_PROFILE_VELOCITY, 80);
    write4(rgrip_id_,  ADDR_PROFILE_ACCEL,    20);
    write4(rwrist_id_, ADDR_PROFILE_ACCEL,    20);
    write1(rgrip_id_,  ADDR_TORQUE_ENABLE, 1);
    write1(rwrist_id_, ADDR_TORQUE_ENABLE, 1);

    std::string topic = get_parameter("error_topic").as_string();
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      topic, 10, std::bind(&DxlNode::cb, this, _1));

    timer_ = create_wall_timer(20ms, std::bind(&DxlNode::update, this));
  }

  ~DxlNode() override {
    write1(rgrip_id_,  ADDR_TORQUE_ENABLE, 0);
    write1(rwrist_id_, ADDR_TORQUE_ENABLE, 0);
    port_->closePort();
  }

private:
  static double clip(double x, double lo, double hi){ return std::max(lo, std::min(hi, x)); }

  void cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    double rgrip_min  = get_parameter("rgrip_min_deg").as_double();
    double rgrip_max  = get_parameter("rgrip_max_deg").as_double();
    double rwrist_min = get_parameter("rwrist_min_deg").as_double();
    double rwrist_max = get_parameter("rwrist_max_deg").as_double();
    bool invert_rgrip = get_parameter("invert_rgrip").as_bool();
    bool invert_rwrist = get_parameter("invert_rwrist").as_bool();
    double max_step = get_parameter("max_step_deg").as_double();
    (void)get_parameter("kp"); // retained for compatibility

    if (msg->data.size() < 2) return;
    double rgrip_cmd  = msg->data[0];
    double rwrist_cmd = msg->data[1];

    (void)max_step;
    double rgrip_target  = invert_rgrip  ? -rgrip_cmd  : rgrip_cmd;
    double rwrist_target = invert_rwrist ? -rwrist_cmd : rwrist_cmd;
    target_rgrip_deg_  = clip(rgrip_target,  rgrip_min,  rgrip_max);
    target_rwrist_deg_ = clip(rwrist_target, rwrist_min, rwrist_max);
  }

  void update(){
    double rgrip_off  = get_parameter("rgrip_offset_deg").as_double();
    double rwrist_off = get_parameter("rwrist_offset_deg").as_double();
    double max_step = get_parameter("max_step_deg").as_double();
    double rgrip_min  = get_parameter("rgrip_min_deg").as_double();
    double rgrip_max  = get_parameter("rgrip_max_deg").as_double();
    double rwrist_min = get_parameter("rwrist_min_deg").as_double();
    double rwrist_max = get_parameter("rwrist_max_deg").as_double();

    auto step_limited = [max_step](double step){
      return std::max(-max_step, std::min(max_step, step));
    };
    auto clip = [](double x, double lo, double hi){
      return std::max(lo, std::min(hi, x));
    };

    double rgrip_step  = step_limited(target_rgrip_deg_  - rgrip_deg_);
    double rwrist_step = step_limited(target_rwrist_deg_ - rwrist_deg_);

    rgrip_deg_  = clip(rgrip_deg_  + rgrip_step,  rgrip_min,  rgrip_max);
    rwrist_deg_ = clip(rwrist_deg_ + rwrist_step, rwrist_min, rwrist_max);

    int rgrip_ticks  = static_cast<int>((rgrip_deg_  + rgrip_off + 180.0) * TICKS_PER_DEG);
    int rwrist_ticks = static_cast<int>((rwrist_deg_ + rwrist_off + 180.0) * TICKS_PER_DEG);
    rgrip_ticks = clamp_ticks(rgrip_ticks, rgrip_info_);
    rwrist_ticks = clamp_ticks(rwrist_ticks, rwrist_info_);

    write4(rgrip_id_,  ADDR_GOAL_POSITION, rgrip_ticks);
    write4(rwrist_id_, ADDR_GOAL_POSITION, rwrist_ticks);
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
  JointInfo rgrip_info_;
  JointInfo rwrist_info_;
  int rgrip_id_{-1};
  int rwrist_id_{-1};
  double rgrip_deg_{0.0}, rwrist_deg_{0.0};
  double target_rgrip_deg_{0.0}, target_rwrist_deg_{0.0};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxlNode>());
  rclcpp::shutdown();
  return 0;
}
