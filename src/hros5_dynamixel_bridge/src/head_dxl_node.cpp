#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

constexpr int PAN_ID  = 19;
constexpr int TILT_ID = 20;
constexpr double TICKS_PER_DEG = 4095.0 / 360.0;

constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_PROFILE_ACCEL    = 108;
constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
constexpr uint16_t ADDR_GOAL_POSITION    = 116;

class DxlNode : public rclcpp::Node {
public:
  DxlNode() : rclcpp::Node("head_dxl_node") {
    declare_parameter<std::string>("device", "/dev/dxl");
    declare_parameter<int>("baud", 1000000);
    declare_parameter<double>("kp", 0.6);
    declare_parameter<double>("max_step_deg", 2.0);
    declare_parameter<double>("pan_min_deg", -120.0);
    declare_parameter<double>("pan_max_deg", 120.0);
    declare_parameter<double>("tilt_min_deg", -30.0);
    declare_parameter<double>("tilt_max_deg", 60.0);
    declare_parameter<double>("pan_offset_deg", 0.0);
    declare_parameter<double>("tilt_offset_deg", 0.0);
    declare_parameter<bool>("invert_pan", false);
    declare_parameter<bool>("invert_tilt", false);
    declare_parameter<std::string>("error_topic", "/hros5/head/target_angles_deg");

    std::string dev = get_parameter("device").as_string();
    int baud = get_parameter("baud").as_int();

    port_ = dynamixel::PortHandler::getPortHandler(dev.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!port_->openPort()) throw std::runtime_error("openPort failed");
    if (!port_->setBaudRate(baud)) throw std::runtime_error("setBaudRate failed");

    write1(PAN_ID,  ADDR_TORQUE_ENABLE, 0);
    write1(TILT_ID, ADDR_TORQUE_ENABLE, 0);
    write1(PAN_ID,  ADDR_OPERATING_MODE, 3);
    write1(TILT_ID, ADDR_OPERATING_MODE, 3);
    write4(PAN_ID,  ADDR_PROFILE_VELOCITY, 80);
    write4(TILT_ID, ADDR_PROFILE_VELOCITY, 80);
    write4(PAN_ID,  ADDR_PROFILE_ACCEL,    20);
    write4(TILT_ID, ADDR_PROFILE_ACCEL,    20);
    write1(PAN_ID,  ADDR_TORQUE_ENABLE, 1);
    write1(TILT_ID, ADDR_TORQUE_ENABLE, 1);

    std::string topic = get_parameter("error_topic").as_string();
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      topic, 10, std::bind(&DxlNode::cb, this, _1));

    timer_ = create_wall_timer(20ms, std::bind(&DxlNode::update, this));
  }

  ~DxlNode() override {
    write1(PAN_ID,  ADDR_TORQUE_ENABLE, 0);
    write1(TILT_ID, ADDR_TORQUE_ENABLE, 0);
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

    int pan_ticks  = static_cast<int>((pan_deg_  + pan_off + 180.0) * TICKS_PER_DEG);
    int tilt_ticks = static_cast<int>((tilt_deg_ + tilt_off + 180.0) * TICKS_PER_DEG);

    write4(PAN_ID,  ADDR_GOAL_POSITION, pan_ticks);
    write4(TILT_ID, ADDR_GOAL_POSITION, tilt_ticks);
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
  double pan_deg_{0.0}, tilt_deg_{0.0};
  double target_pan_deg_{0.0}, target_tilt_deg_{0.0};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxlNode>());
  rclcpp::shutdown();
  return 0;
}
