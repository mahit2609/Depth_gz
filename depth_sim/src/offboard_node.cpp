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
            "/fmu/out/vehicle_status_v1", 10,  // Changed to vehicle_status_v1
            std::bind(&OffboardControlNode::vehicle_status_callback, this, std::placeholders::_1));
            
        vehicle_command_ack_sub_ = this->create_subscription<msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack", 10,
            std::bind(&OffboardControlNode::command_ack_callback, this, std::placeholders::_1));
        
        // Timer callback
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OffboardControlNode::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Offboard control node started (depth_sim pkg).");
    }

private:
    void timer_callback()
    {
        auto now = this->get_clock()->now();
        uint64_t timestamp = now.nanoseconds() / 1000;
        
        // Must continuously publish these
        publish_offboard_mode(timestamp);
        publish_trajectory_setpoint(timestamp);
        
        // Print debug info every second (10 calls at 100ms)
        if (count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Sending commands. Count: %d", count_);
        }
        
        // Send initialization commands in sequence
        if (count_ == 5) {
            send_preflight_check_override(timestamp);
        }
        if (count_ == 7) {
            override_safety_switch(timestamp);
        }
        if (count_ == 8) {
            set_manual_mode(timestamp);
        }
        
        // Try to arm if not armed yet (every 2 seconds)
        if (!armed_ && count_ > 10 && count_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "Sending arm command");
            send_arm_command(timestamp);
        }
        
        // Try to set to offboard mode if armed but not in offboard mode (every 2 seconds)
        // PX4 offboard mode is typically nav_state 14
        if (armed_ && current_nav_state_ != 14 && count_ % 20 == 0) {
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
        }
    }
    
    void command_ack_callback(const msg::VehicleCommandAck::SharedPtr msg)
    {
        // Common result codes
        // 0: accepted/OK
        // 1: temporarily rejected
        // 2: denied
        // 3: unsupported
        // 4: failed
        // 5: in progress
        
        std::string result;
        switch(msg->result) {
            case 0: result = "ACCEPTED"; break;
            case 1: result = "TEMPORARILY REJECTED"; break;
            case 2: result = "DENIED"; break;
            case 3: result = "UNSUPPORTED"; break;
            case 4: result = "FAILED"; break;
            case 5: result = "IN PROGRESS"; break;
            default: result = "UNKNOWN";
        }
        
        RCLCPP_INFO(this->get_logger(), "Command ACK - Command: %d, Result: %s (%d)",
                    msg->command, result.c_str(), msg->result);
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
        msg.position[0] = 0.0;     // x position
        msg.position[1] = 0.0;     // y position
        msg.position[2] = -2.0;    // z position (NED frame, negative value = altitude)
        msg.yaw = 0.0;             // yaw angle
        traj_setpoint_pub_->publish(msg);
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
    
    void send_preflight_check_override(uint64_t timestamp)
    {
        msg::VehicleCommand msg;
        msg.timestamp = timestamp;
        msg.command = msg::VehicleCommand::VEHICLE_CMD_DO_SET_PARAMETER;
        msg.param1 = 22;  // COM_ARM_AUTH_REQ parameter
        msg.param2 = 0;   // Parameter value (0 = disable auth requirement)
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Sending command to bypass preflight checks");
    }
    
    void override_safety_switch(uint64_t timestamp)
    {
        msg::VehicleCommand msg;
        msg.timestamp = timestamp;
        msg.command = msg::VehicleCommand::VEHICLE_CMD_DO_SET_PARAMETER;
        msg.param1 = 55;  // CBRK_IO_SAFETY parameter
        msg.param2 = 22027;  // Magic number to disable safety switch
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Sending command to override safety switch");
    }
    
    void set_manual_mode(uint64_t timestamp)
    {
        msg::VehicleCommand msg;
        msg.timestamp = timestamp;
        msg.param1 = 1.0;  // 1 = main mode
        msg.param2 = 0.0;  // 0 = MANUAL mode
        msg.command = msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Setting manual mode");
    }

    // Publishers
    rclcpp::Publisher<msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    
    // Subscribers
    rclcpp::Subscription<msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
    
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