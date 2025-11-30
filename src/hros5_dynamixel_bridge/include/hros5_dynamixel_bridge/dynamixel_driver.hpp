#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hros5_dynamixel_bridge/dxl_bus_config.hpp"

struct JointConfig
{
    std::string name;
    int id;
    std::string model;
    int cw_limit;
    int ccw_limit;
};

class DynamixelDriver
{
public:
    DynamixelDriver(const DxlBusConfig& bus_config);
    ~DynamixelDriver();

    bool load_config(const std::string& yaml_path);
    bool enable_torque_all();
    bool disable_torque_all();
    bool sync_write_positions(const std::unordered_map<std::string, double>& position_map);
    bool sync_read_positions(std::unordered_map<std::string, double>& position_map);

    std::vector<std::string> get_joint_names() const;
    std::unordered_map<std::string, JointConfig> joint_configs_;

private:
    DxlBusConfig bus_config_;
    std::string port_;
    int baudrate_;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    std::vector<uint8_t> ids_;
    std::vector<std::string> joint_names_;

    // Helper
    template<typename Fn>
    bool with_retry(const char* label, Fn&& fn) const {
        for (int attempt = 0; attempt <= bus_config_.retries; ++attempt) {
            if (bus_config_.packet_timeout_ms > 0.0) {
                portHandler_->setPacketTimeout(bus_config_.packet_timeout_ms);
            }
            if (fn()) {
                return true;
            }
        }
        std::cerr << label << " failed after "
                  << bus_config_.retries + 1 << " attempt(s)" << std::endl;
        return false;
    }
    int get_raw_position(const std::string& joint, double rad) const;
    double get_rad_position(const std::string& joint, int raw) const;
};
// Implementation of DynamixelDriver methods
