#include "hros5_dynamixel_bridge/dynamixel_driver.hpp"
#include <iostream>
#include <cmath>
#include <limits>

// Control table addresses (Protocol 2.0, for MX-series)
constexpr int ADDR_TORQUE_ENABLE      = 64;
constexpr int ADDR_GOAL_POSITION      = 116;
constexpr int ADDR_PRESENT_POSITION   = 132;
constexpr int TORQUE_ENABLE           = 1;
constexpr int TORQUE_DISABLE          = 0;

DynamixelDriver::DynamixelDriver(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate)
{
    portHandler_ = dynamixel::PortHandler::getPortHandler(port_.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!portHandler_->openPort())
        std::cerr << "Failed to open port " << port_ << std::endl;
    if (!portHandler_->setBaudRate(baudrate_))
        std::cerr << "Failed to set baudrate " << baudrate_ << std::endl;
}

DynamixelDriver::~DynamixelDriver()
{
    disable_torque_all();
    portHandler_->closePort();
}

bool DynamixelDriver::load_config(const std::string& yaml_path)
{
    YAML::Node config = YAML::LoadFile(yaml_path);
    joint_configs_.clear();
    ids_.clear();
    joint_names_.clear();
    if (!config["joints"]) {
        std::cerr << "YAML config missing 'joints' key!" << std::endl;
        return false;
    }
    for (const auto& node : config["joints"]) {
        JointConfig jc;
        jc.name = node["name"].as<std::string>();
        jc.id = node["id"].as<int>();
        jc.model = node["model"] ? node["model"].as<std::string>() : "MX-28";
        // Use -1 for limits if TBD or missing
        jc.cw_limit = (node["cw_limit"] && node["cw_limit"].IsScalar() && node["cw_limit"].as<std::string>() != "TBD")
                          ? node["cw_limit"].as<int>() : -1;
        jc.ccw_limit = (node["ccw_limit"] && node["ccw_limit"].IsScalar() && node["ccw_limit"].as<std::string>() != "TBD")
                          ? node["ccw_limit"].as<int>() : -1;

        joint_configs_[jc.name] = jc;
        ids_.push_back(jc.id);
        joint_names_.push_back(jc.name);
    }
    return true;
}

bool DynamixelDriver::enable_torque_all()
{
    for (const auto& j : joint_configs_) {
        int dxl_comm_result = packetHandler_->write1ByteTxRx(
            portHandler_, j.second.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "Failed to enable torque for " << j.second.name << std::endl;
    }
    return true;
}

bool DynamixelDriver::disable_torque_all()
{
    for (const auto& j : joint_configs_) {
        packetHandler_->write1ByteTxRx(
            portHandler_, j.second.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    }
    return true;
}

bool DynamixelDriver::sync_write_positions(const std::unordered_map<std::string, double>& position_map)
{
    dynamixel::GroupSyncWrite syncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION, 4);
    for (const auto& j : joint_configs_) {
        auto it = position_map.find(j.first);
        if (it == position_map.end()) continue;
        int raw = get_raw_position(j.first, it->second);
        uint8_t param_goal_position[4] = {
            DXL_LOBYTE(DXL_LOWORD(raw)),
            DXL_HIBYTE(DXL_LOWORD(raw)),
            DXL_LOBYTE(DXL_HIWORD(raw)),
            DXL_HIBYTE(DXL_HIWORD(raw))
        };
        syncWrite.addParam(j.second.id, param_goal_position);
    }
    bool result = syncWrite.txPacket() == COMM_SUCCESS;
    syncWrite.clearParam();
    return result;
}

bool DynamixelDriver::sync_read_positions(std::unordered_map<std::string, double>& position_map)
{
    dynamixel::GroupSyncRead syncRead(portHandler_, packetHandler_, ADDR_PRESENT_POSITION, 4);
    for (const auto& j : joint_configs_) {
        syncRead.addParam(j.second.id);
    }
    if (syncRead.txRxPacket() != COMM_SUCCESS) {
        std::cerr << "SyncRead failed!" << std::endl;
        return false;
    }
    for (const auto& j : joint_configs_) {
        if (!syncRead.isAvailable(j.second.id, ADDR_PRESENT_POSITION, 4))
            continue;
        int32_t pos = syncRead.getData(j.second.id, ADDR_PRESENT_POSITION, 4);
        position_map[j.first] = get_rad_position(j.first, pos);
    }
    return true;
}

std::vector<std::string> DynamixelDriver::get_joint_names() const
{
    return joint_names_;
}

// --- Helper functions ---

int DynamixelDriver::get_raw_position(const std::string& joint, double rad) const
{
    // Default: 0-4095 for MX-series
    int max_raw = 4095, min_raw = 0;
    auto it = joint_configs_.find(joint);
    if (it != joint_configs_.end()) {
        // Optionally, support other models with different resolution here
        // For now, assume all are MX Protocol 2.0 12-bit (0-4095)
        // (MX-106/MX-64/MX-28/MX-12W all use 0-4095 for position)
        if (it->second.cw_limit >= 0 && it->second.ccw_limit >= 0) {
            min_raw = it->second.cw_limit;
            max_raw = it->second.ccw_limit;
        }
    }
    double rad_min = -3.14159;
    double rad_max = 3.14159;
    int val = int(std::round((rad - rad_min) * (max_raw - min_raw) / (rad_max - rad_min) + min_raw));
    if (val < min_raw) val = min_raw;
    if (val > max_raw) val = max_raw;
    return val;
}

double DynamixelDriver::get_rad_position(const std::string& joint, int raw) const
{
    int max_raw = 4095, min_raw = 0;
    auto it = joint_configs_.find(joint);
    if (it != joint_configs_.end()) {
        if (it->second.cw_limit >= 0 && it->second.ccw_limit >= 0) {
            min_raw = it->second.cw_limit;
            max_raw = it->second.ccw_limit;
        }
    }
    double rad_min = -3.14159;
    double rad_max = 3.14159;
    double val = rad_min + (raw - min_raw) * (rad_max - rad_min) / (max_raw - min_raw);
    return val;
}
// Note: This implementation assumes all joints use the same range and resolution.
// If different models are used, you may need to adjust the limits and resolution accordingly.