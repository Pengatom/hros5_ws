#pragma once

#include <stdexcept>
#include <string>
#include <filesystem>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

struct DxlBusConfig {
  std::string device;
  int baud{0};
  double protocol_version{2.0};
  double packet_timeout_ms{0.0};
  int retries{0};
};

struct DxlBusDefaults {
  std::string device{"/dev/dxl"};
  int baud{1000000};
  double protocol_version{2.0};
  double packet_timeout_ms{100.0};
  int retries{1};
  std::string config_path;
};

inline std::string default_bus_config_path(){
  try {
    auto share = ament_index_cpp::get_package_share_directory("hros5_dynamixel_bridge");
    return (std::filesystem::path(share) / "config" / "dxl_bus.yaml").string();
  } catch (const std::exception&) {
    return std::string();
  }
}

inline DxlBusDefaults load_bus_defaults(const std::string& config_file){
  DxlBusDefaults defaults;
  defaults.config_path = config_file;
  if (config_file.empty()) {
    return defaults;
  }
  try {
    YAML::Node yaml = YAML::LoadFile(config_file);
    auto params = yaml["/**"]["ros__parameters"];
    if (params) {
      if (params["device"]) defaults.device = params["device"].as<std::string>();
      if (params["baud"]) defaults.baud = params["baud"].as<int>();
      if (params["protocol_version"]) defaults.protocol_version = params["protocol_version"].as<double>();
      if (params["bus_timeout_ms"]) defaults.packet_timeout_ms = params["bus_timeout_ms"].as<double>();
      if (params["bus_retries"]) defaults.retries = params["bus_retries"].as<int>();
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: failed to load bus config defaults from " << config_file
              << " (" << e.what() << ")" << std::endl;
  }
  return defaults;
}

inline DxlBusConfig declare_and_get_bus_config(rclcpp::Node& node) {
  std::string default_file = default_bus_config_path();
  auto defaults = load_bus_defaults(default_file);

  std::string bus_file = node.declare_parameter<std::string>("bus_config_file", default_file);
  if (!bus_file.empty() && bus_file != default_file) {
    defaults = load_bus_defaults(bus_file);
  }

  DxlBusConfig cfg;
  cfg.device = node.declare_parameter<std::string>("device", defaults.device);
  if (cfg.device.empty()) {
    cfg.device = node.declare_parameter<std::string>("port", defaults.device);
  }

  cfg.baud = node.declare_parameter<int>("baud", defaults.baud);
  if (cfg.baud == 0) {
    cfg.baud = node.declare_parameter<int>("baudrate", defaults.baud);
  }

  cfg.protocol_version = node.declare_parameter<double>("protocol_version", defaults.protocol_version);
  cfg.packet_timeout_ms = node.declare_parameter<double>("bus_timeout_ms", defaults.packet_timeout_ms);
  cfg.retries = node.declare_parameter<int>("bus_retries", defaults.retries);

  if (cfg.device.empty()) {
    throw std::runtime_error("device parameter is required for the Dynamixel bus");
  }
  if (cfg.baud <= 0) {
    throw std::runtime_error("baud parameter must be set for the Dynamixel bus");
  }
  if (cfg.protocol_version <= 0.0) {
    throw std::runtime_error("protocol_version must be greater than zero");
  }
  return cfg;
}

template<typename Fn>
inline bool dxl_with_retry(
  const DxlBusConfig& cfg,
  dynamixel::PortHandler* port,
  const rclcpp::Logger& logger,
  const char* label,
  Fn&& fn)
{
  for (int attempt = 0; attempt <= cfg.retries; ++attempt) {
    if (cfg.packet_timeout_ms > 0.0 && port) {
      port->setPacketTimeout(cfg.packet_timeout_ms);
    }
    uint8_t err = 0;
    int res = fn(err);
    if (res == COMM_SUCCESS && err == 0) {
      return true;
    }
    if (attempt == cfg.retries) {
      RCLCPP_WARN(
        logger, "%s failed (res=%d, err=%d) after %d attempt(s)",
        label, res, err, attempt + 1);
    }
  }
  return false;
}
