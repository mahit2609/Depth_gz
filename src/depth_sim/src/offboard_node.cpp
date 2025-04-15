#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include "stair_follower.hpp"  // Your custom stair follower class

using namespace px4_msgs;

class OffboardControlNode : public rclcpp::Node
{
public:
    OffboardControlNode() : Node("offboard_control_node"), count_(0), current_x_(0.0),
                            stair_follower_(1.5, 0.4) // cruise height, max step height
    {
        vehicle_command_pub_ = this->create_publisher<msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        offboard_mode_pub_ = this->create_publisher<msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        traj_setpoint_pub_ = this->create_publisher<msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        vehicle_status_sub_ = this->create_subscription<msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10,
            std::bind(&OffboardControlNode::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_local_pos_sub_ = this->create_subscription<msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 10,
            std::bind(&OffboardControlNode::local_position_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&OffboardControlNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Offboard control node started.");
    }

private:
    void timer_callback()
    {
        auto now = this->get_clock()->now();
        uint64_t timestamp = now.nanoseconds() / 1000;

        publish_offboard_mode(timestamp);
        publish_trajectory_setpoint(timestamp);

        if (count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Publishing setpoint. Count: %d", count_);
        }

        if (count_ == 10) {
            RCLCPP_INFO(this->get_logger(), "Sending arm command");
            send_arm_command(timestamp);
        } else if (count_ == 50) {
            RCLCPP_INFO(this->get_logger(), "Setting offboard mode");
            set_offboard_mode(timestamp);
        }

        count_++;
    }

    void publish_offboard_mode(uint64_t timestamp)
    {
        msg::OffboardControlMode msg;
        msg.timestamp = timestamp;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(uint64_t timestamp)
    {
        msg::TrajectorySetpoint msg;
        msg.timestamp = timestamp;

        if (!stair_follower_.isTerrainValid()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Terrain data not yet valid.");
            return;
        }

        current_x_ += 1.0; 
        double terrain_z = stair_follower_.getTerrainZ();
        double target_z = stair_follower_.computeTargetZ(terrain_z);

        msg.position[0] = current_x_;
        msg.position[1] = 0.0;
        msg.position[2] = -target_z; // NED frame (z is positive down)
        msg.yaw = 0.0;

        traj_setpoint_pub_->publish(msg);
    }

    void send_arm_command(uint64_t timestamp)
    {
        msg::VehicleCommand msg;
        msg.timestamp = timestamp;
        msg.param1 = 1.0; // 1 = arm
        msg.command = msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    void set_offboard_mode(uint64_t timestamp)
    {
        msg::VehicleCommand msg;
        msg.timestamp = timestamp;
        msg.param1 = 1.0;
        msg.param2 = 6.0; // 6 = OFFBOARD
        msg.command = msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    void vehicle_status_callback(const msg::VehicleStatus::SharedPtr msg)
    {
        if (current_nav_state_ != msg->nav_state || armed_ != (msg->arming_state == msg->ARMING_STATE_ARMED)) {
            current_nav_state_ = msg->nav_state;
            armed_ = msg->arming_state == msg->ARMING_STATE_ARMED;
            RCLCPP_INFO(this->get_logger(), "Vehicle state changed - Nav state: %d, Armed: %s", 
                        current_nav_state_, armed_ ? "true" : "false");
        }
    }

    void local_position_callback(const msg::VehicleLocalPosition::SharedPtr msg)
    {
        stair_follower_.updateFromLocalPosition(*msg);
    }

    // Publishers
    rclcpp::Publisher<msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;

    // Subscriptions
    rclcpp::Subscription<msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Internal state
    int count_;
    double current_x_;
    uint8_t current_nav_state_ = 0;
    bool armed_ = false;

    // Stair following logic
    StairFollower stair_follower_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlNode>());
    rclcpp::shutdown();
    return 0;
}
