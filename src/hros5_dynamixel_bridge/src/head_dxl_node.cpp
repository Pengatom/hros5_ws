#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "hros5_dynamixel_bridge/dxl_bus_config.hpp"
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <unordered_map>
#include <sstream>
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
    throw std::runtime_error("head servos config missing 'joints' key");
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

} // namespace

class DxlNode : public rclcpp::Node {
public:
  DxlNode() : rclcpp::Node("head_dxl_node") {
    bus_config_ = declare_and_get_bus_config(*this);
    declare_parameter<double>("kp", 0.6);
    declare_parameter<double>("max_step_deg", 2.0);
    declare_parameter<std::string>("config_package", "hros5_control");
    declare_parameter<std::string>("config_file", "config/joints/head.yaml");
    declare_parameter<bool>("invert_pan", false);
    declare_parameter<bool>("invert_tilt", false);
    declare_parameter<std::string>("echo_request_topic", "/hros5/head/echo_positions");
    declare_parameter<std::string>("error_topic", "/hros5/head/target_angles_deg");

    std::string config_path = resolve_config_path(
      get_parameter("config_file").as_string(),
      get_parameter("config_package").as_string());
    auto joints = load_joint_info(config_path);
    auto yaw_it = joints.find("HeadYaw");
    auto pitch_it = joints.find("HeadPitch");
    if (yaw_it == joints.end() || pitch_it == joints.end()) {
      throw std::runtime_error("HeadYaw or HeadPitch missing from head servos config");
    }
    pan_info_ = yaw_it->second;
    tilt_info_ = pitch_it->second;
    pan_id_ = pan_info_.id;
    tilt_id_ = tilt_info_.id;

    double pan_min_default = pan_info_.cw_limit >= 0 ? ticks_to_deg(pan_info_.cw_limit) : -120.0;
    double pan_max_default = pan_info_.ccw_limit >= 0 ? ticks_to_deg(pan_info_.ccw_limit) : 120.0;
    double tilt_min_default = tilt_info_.cw_limit >= 0 ? ticks_to_deg(tilt_info_.cw_limit) : -30.0;
    double tilt_max_default = tilt_info_.ccw_limit >= 0 ? ticks_to_deg(tilt_info_.ccw_limit) : 60.0;

    declare_parameter<double>("pan_min_deg", pan_min_default);
    declare_parameter<double>("pan_max_deg", pan_max_default);
    declare_parameter<double>("tilt_min_deg", tilt_min_default);
    declare_parameter<double>("tilt_max_deg", tilt_max_default);
    declare_parameter<double>("pan_offset_deg", pan_info_.offset_deg);
    declare_parameter<double>("tilt_offset_deg", tilt_info_.offset_deg);

    auto bus_cfg_param = get_parameter("bus_config").as_string();
    auto bus_cfg_file_param = get_parameter("bus_config_file").as_string();
    std::string bus_cfg_path = resolve_bus_config_path(
      bus_cfg_file_param.empty() ? bus_cfg_param : bus_cfg_file_param);
    RCLCPP_INFO(this->get_logger(),
      "Bus config: %s (device=%s, baud=%d, protocol=%.1f, timeout_ms=%.1f, retries=%d)",
      bus_cfg_path.c_str(), bus_config_.device.c_str(), bus_config_.baud,
      bus_config_.protocol_version, bus_config_.packet_timeout_ms, bus_config_.retries);

    auto config_pkg = get_parameter("config_package").as_string();
    auto config_file = get_parameter("config_file").as_string();
    RCLCPP_INFO(this->get_logger(),
      "Head joint config: package=%s file=%s (resolved: %s)",
      config_pkg.c_str(), config_file.c_str(), config_path.c_str());

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

    write1(pan_id_,  ADDR_TORQUE_ENABLE, 0);
    write1(tilt_id_, ADDR_TORQUE_ENABLE, 0);
    write1(pan_id_,  ADDR_OPERATING_MODE, 3);
    write1(tilt_id_, ADDR_OPERATING_MODE, 3);
    write4(pan_id_,  ADDR_PROFILE_VELOCITY, 80);
    write4(tilt_id_, ADDR_PROFILE_VELOCITY, 80);
    write4(pan_id_,  ADDR_PROFILE_ACCEL,    20);
    write4(tilt_id_, ADDR_PROFILE_ACCEL,    20);
    write1(pan_id_,  ADDR_TORQUE_ENABLE, 1);
    write1(tilt_id_, ADDR_TORQUE_ENABLE, 1);

    std::string topic = get_parameter("error_topic").as_string();
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      topic, 10, std::bind(&DxlNode::cb, this, _1));
    echo_sub_ = create_subscription<std_msgs::msg::Empty>(
      get_parameter("echo_request_topic").as_string(), 10,
      std::bind(&DxlNode::handle_echo_request, this, std::placeholders::_1));

    timer_ = create_wall_timer(20ms, std::bind(&DxlNode::update, this));
  }

  ~DxlNode() override {
    write1(pan_id_,  ADDR_TORQUE_ENABLE, 0);
    write1(tilt_id_, ADDR_TORQUE_ENABLE, 0);
    port_->closePort();
  }

private:
  static double clip(double x, double lo, double hi){ return std::max(lo, std::min(hi, x)); }

  void cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    double pan_min  = get_parameter("pan_min_deg").as_double();
    double pan_max  = get_parameter("pan_max_deg").as_double();
    double tilt_min = get_parameter("tilt_min_deg").as_double();
    double tilt_max = get_parameter("tilt_max_deg").as_double();
    double pan_off  = get_parameter("pan_offset_deg").as_double();
    double tilt_off = get_parameter("tilt_offset_deg").as_double();
    bool invert_pan = get_parameter("invert_pan").as_bool();
    bool invert_tilt = get_parameter("invert_tilt").as_bool();
    double max_step = get_parameter("max_step_deg").as_double();
    (void)get_parameter("kp"); // parameter retained for compatibility, no longer used

    if (msg->data.size() < 2) return;
    double pan_cmd  = msg->data[0];  // absolute target pan (deg)
    double tilt_cmd = msg->data[1];  // absolute target tilt (deg)

    (void)max_step; // consumed in update()
    double pan_target  = invert_pan  ? -pan_cmd  : pan_cmd;
    double tilt_target = invert_tilt ? -tilt_cmd : tilt_cmd;
    target_pan_deg_  = clip(pan_target,  pan_min,  pan_max);
    target_tilt_deg_ = clip(tilt_target, tilt_min, tilt_max);
  }

  void update(){
    double pan_off  = get_parameter("pan_offset_deg").as_double();
    double tilt_off = get_parameter("tilt_offset_deg").as_double();
    double max_step = get_parameter("max_step_deg").as_double();
    double pan_min  = get_parameter("pan_min_deg").as_double();
    double pan_max  = get_parameter("pan_max_deg").as_double();
    double tilt_min = get_parameter("tilt_min_deg").as_double();
    double tilt_max = get_parameter("tilt_max_deg").as_double();

    auto step_limited = [max_step](double step){
      return std::max(-max_step, std::min(max_step, step));
    };
    auto clip = [](double x, double lo, double hi){
      return std::max(lo, std::min(hi, x));
    };

    double pan_step  = step_limited(target_pan_deg_  - pan_deg_);
    double tilt_step = step_limited(target_tilt_deg_ - tilt_deg_);

    pan_deg_  = clip(pan_deg_  + pan_step,  pan_min,  pan_max);
    tilt_deg_ = clip(tilt_deg_ + tilt_step, tilt_min, tilt_max);

    int pan_ticks  = clamp_ticks(static_cast<int>((pan_deg_  + pan_off + 180.0) * TICKS_PER_DEG), pan_info_);
    int tilt_ticks = clamp_ticks(static_cast<int>((tilt_deg_ + tilt_off + 180.0) * TICKS_PER_DEG), tilt_info_);

    write4(pan_id_,  ADDR_GOAL_POSITION, pan_ticks);
    write4(tilt_id_, ADDR_GOAL_POSITION, tilt_ticks);
  }

  void handle_echo_request(const std_msgs::msg::Empty::SharedPtr) {
    std::ostringstream polling;
    polling << "Polling servos: " << pan_id_ << ", " << tilt_id_;
    RCLCPP_INFO(this->get_logger(), "%s", polling.str().c_str());

    char time_buf[9] = {0};
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &tm);

    struct EchoItem { int id; const char* label; };
    std::array<EchoItem, 2> items{{{pan_id_, "HeadYaw"}, {tilt_id_, "HeadPitch"}}};

    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary.precision(2);

    for (const auto& item : items) {
      uint32_t pos = 0;
      std::string label = std::string("echo read id=") + std::to_string(item.id);
      bool ok = dxl_with_retry(
        bus_config_, port_, this->get_logger(), label.c_str(),
        [&](uint8_t& err){
          return packet_->read4ByteTxRx(
            port_, static_cast<uint8_t>(item.id), ADDR_PRESENT_POSITION, &pos, &err);
        });
      if (!ok) {
        summary << "[" << item.label << " id " << item.id << ": read failed] ";
        continue;
      }
      int ticks = static_cast<int>(pos);
      double deg = ticks_to_deg(ticks);
      summary << item.label << " (" << item.id << "): " << deg << " deg; ";

      std::ostringstream detail;
      detail.setf(std::ios::fixed);
      detail << "[" << time_buf << "] ID "
             << std::setw(3) << item.id << ": "
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
  JointInfo pan_info_;
  JointInfo tilt_info_;
  int pan_id_{-1};
  int tilt_id_{-1};
  double pan_deg_{0.0}, tilt_deg_{0.0};
  double target_pan_deg_{0.0}, target_tilt_deg_{0.0};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxlNode>());
  rclcpp::shutdown();
  return 0;
}
