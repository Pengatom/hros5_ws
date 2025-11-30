#include "hros5_dynamixel_bridge/dynamixel_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <iostream>
#include <chrono>
#include <unordered_map>
#include <string>
#include <vector>
#include <memory>

using namespace std::chrono_literals;

class DynamixelBridgeNode : public rclcpp::Node
{
public:
    DynamixelBridgeNode()
    : Node("dynamixel_bridge_node")
    {
        bus_config_ = declare_and_get_bus_config(*this);
        declare_parameter<std::string>("config_file", "config/joints.yaml");
        config_file_ = get_parameter("config_file").as_string();

        driver_ = std::make_unique<DynamixelDriver>(bus_config_);
        if (!driver_->load_config(config_file_))
            RCLCPP_FATAL(get_logger(), "Failed to load YAML config!");
        else
            RCLCPP_INFO(get_logger(), "Loaded config: %s", config_file_.c_str());

        driver_->enable_torque_all();

        joint_names_ = driver_->get_joint_names();
        pub_joint_state_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        sub_traj_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "joint_trajectory", 10,
            std::bind(&DynamixelBridgeNode::traj_callback, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(20ms, std::bind(&DynamixelBridgeNode::timer_callback, this));
    }

    ~DynamixelBridgeNode() override
    {
        driver_->disable_torque_all();
    }

private:
    void timer_callback()
    {
        // Sync read all positions, publish as JointState
        std::unordered_map<std::string, double> positions;
        if (driver_->sync_read_positions(positions))
        {
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = now();
            msg.name = joint_names_;
            msg.position.reserve(joint_names_.size());
            for (const auto& n : joint_names_)
                msg.position.push_back(positions[n]);
            pub_joint_state_->publish(msg);
        }
    }

    void traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty()) return;
        // Use only the first point (simple implementation, supports multi-joint)
        std::unordered_map<std::string, double> pos_map;
        for (size_t i = 0; i < msg->joint_names.size(); ++i)
        {
            if (i < msg->points[0].positions.size())
                pos_map[msg->joint_names[i]] = msg->points[0].positions[i];
        }
        driver_->sync_write_positions(pos_map);
    }

    // Members
    std::string config_file_;
    DxlBusConfig bus_config_;
    std::vector<std::string> joint_names_;
    std::unique_ptr<DynamixelDriver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_traj_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
