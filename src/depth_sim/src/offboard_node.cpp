#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

using namespace px4_msgs;

class OffboardControlNode : public rclcpp::Node
{
public:
    OffboardControlNode() : Node("offboard_control_node"), count_(0)
    {
        // Publishers
        vehicle_command_pub_ = this->create_publisher<msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        offboard_mode_pub_ = this->create_publisher<msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        traj_setpoint_pub_ = this->create_publisher<msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        
        // Subscribers
        vehicle_status_sub_ = this->create_subscription<msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10,
            std::bind(&OffboardControlNode::vehicle_status_callback, this, std::placeholders::_1));
        
        // Timer callback
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz, matching the Python version
            std::bind(&OffboardControlNode::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Offboard control node started.");
    }

private:
    void timer_callback()
    {
        auto now = this->get_clock()->now();
        uint64_t timestamp = now.nanoseconds() / 1000;
        
        // Must continuously publish offboard control mode
        publish_offboard_mode(timestamp);
        
        // Always publish setpoint
        publish_trajectory_setpoint(timestamp);
        
        // Print debug info every second (50 calls at 20ms)
        if (count_ % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Publishing control modes and setpoints. Count: %d", count_);
        }
        
        // State machine for initialization
        if (count_ == 10) {
            RCLCPP_INFO(this->get_logger(), "Sending arm command");
            send_arm_command(timestamp);
        }
        else if (count_ == 50) {
            RCLCPP_INFO(this->get_logger(), "Setting offboard mode");
            set_offboard_mode(timestamp);
        }
        
        count_++;
    }

    void vehicle_status_callback(const msg::VehicleStatus::SharedPtr msg)
    {
        if (current_nav_state_ != msg->nav_state || armed_ != (msg->arming_state == msg->ARMING_STATE_ARMED)) {
            current_nav_state_ = msg->nav_state;
            armed_ = msg->arming_state == msg->ARMING_STATE_ARMED;
            RCLCPP_INFO(this->get_logger(), "Vehicle state changed - Nav state: %d, Armed: %s", 
                      current_nav_state_, armed_ ? "true" : "false");
            
            // Print if in offboard mode
            if (current_nav_state_ == msg->NAVIGATION_STATE_OFFBOARD) {
                RCLCPP_INFO(this->get_logger(), "Vehicle now in OFFBOARD mode");
            }
        }
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
        msg.position[0] = 2.0;     // x position (meters)
        msg.position[1] = 0.0;     // y position (meters)
        msg.position[2] = -5.0;    // z position 
        msg.yaw = 0.0;             // yaw angle 
        traj_setpoint_pub_->publish(msg);
        //if(count_>10)
        //{
        //    msg.position[0] = 2.0;     // x position (meters)
        //    msg.position[1] = 2.0;     // y position (meters)
        //    msg.position[2] = -5.0;    // z position (NED frame, negative value = altitude in meters)
        //    msg.yaw = 0.0;
        //}
    }

    void send_arm_command(uint64_t timestamp)
    {
        msg::VehicleCommand msg;
        msg.timestamp = timestamp;
        msg.param1 = 1.0;        // 1 = arm, 0 = disarm
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
        msg.param1 = 1.0;        // 1 = offboard mode
        msg.param2 = 6.0;        // 6 = OFFBOARD mode
        msg.command = msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    // Publishers
    rclcpp::Publisher<msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    
    // Subscribers
    rclcpp::Subscription<msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State variables
    int count_;
    uint8_t current_nav_state_ = 0;
    bool armed_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlNode>());
    rclcpp::shutdown();
    return 0;
}