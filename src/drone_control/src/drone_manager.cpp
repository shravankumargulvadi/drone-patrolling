#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp" // Ensure this matches the actual path in your installation
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <chrono>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include "px4_msgs/msg/vehicle_status.hpp"          
#include "px4_msgs/msg/vehicle_command_ack.hpp"     

using namespace std::chrono;
using namespace std::chrono_literals;

// Define any required enums or structs to represent flight states or other constants
enum class FlightState 
{
    WAITING, // Waiting to start
    TAKING_OFF,
    HOVERING, // Taking off complete
    FOLLOWING_TRAJECTORY, // Following the defined trajectory
   
};

struct DroneState 
{
    float posX, posY, posZ;
    float velX, velY, velZ;

};

class DroneControl : public rclcpp::Node {
public:
    DroneControl() : Node("drone_control"), armed_(false) {
        initializeWaypoints();
        generate_trajectory(5);
        // Initialize publishers and subscribers with appropriate topics and QoS settings
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);  

        rclcpp::QoS qos{rclcpp::SensorDataQoS()};
        qos.keep_last(10); // or more if needed
        qos.best_effort(); // maintain SensorDataQoS behavior
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // optional but explicit

        // Create the subscription using the customized QoS
        local_position_subscriber =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position",
                qos,
                std::bind(&DroneControl::local_position_callback,
                        this,
                        std::placeholders::_1));

        offboard_setpoint_counter_ = 0;
        current_state_ = FlightState::WAITING;
        current_waypoint_index_ = 0;
        waypoint_accuracy = 0.5;
        waypoint_reached = false;
        
        kIsFlyingThreshold = 1;

         vehicle_status_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status",
                qos,
                std::bind(&DroneControl::vehicle_status_cb, this, std::placeholders::_1));

		auto timer_callback = [this]() -> void 
        {
        
            current_state_ = get_current_state();
            switch (current_state_) {
                case FlightState::WAITING:
                    //if (std::abs(drone_state.posZ) < kIsFlyingThreshold) return;
                    current_state_ = FlightState::TAKING_OFF;
                    break;
                   
                case FlightState::TAKING_OFF:
                    // Constantly publish setpoints and offboard control mode
                    next_waypoint = {0, 0, -8};  // takeoff waypoint

                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(next_waypoint);

                    if (offboard_setpoint_counter_ == 10) {
                        // After 10 cycles (~500ms), send OFFBOARD mode command and arm
                        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // PX4_NAVIGATION_STATE_OFFBOARD
                        arm();
                    }

                    if (offboard_setpoint_counter_ < 11) {
                        offboard_setpoint_counter_++;
                    }

                    if (waypoint_reached) {
                        current_state_ = FlightState::HOVERING;
                    }

                    RCLCPP_INFO(this->get_logger(), "Taking off...");
                    break;
                    

                case FlightState::HOVERING:
                    // Command the drone to take off
                    // Once takeoff is achieved, move to the next state
                    current_state_ = FlightState::FOLLOWING_TRAJECTORY;
                    break;

                case FlightState::FOLLOWING_TRAJECTORY:
                    // Always publish control mode and setpoint at >2Hz
                    publish_offboard_control_mode();

                    if (current_waypoint_index_ < trajectory_waypoints.size()) {
                        next_waypoint = {
                            trajectory_waypoints[current_waypoint_index_][0],
                            trajectory_waypoints[current_waypoint_index_][1],
                            trajectory_waypoints[current_waypoint_index_][2]
                        };

                        publish_trajectory_setpoint(next_waypoint);

                        RCLCPP_INFO(this->get_logger(), "Set waypoint: %.2f, %.2f, %.2f",
                                    next_waypoint[0], next_waypoint[1], next_waypoint[2]);

                        if (waypoint_reached) {
                            current_waypoint_index_++;
                            RCLCPP_INFO(this->get_logger(), "Waypoint %lu reached. Moving to next.",
                                        current_waypoint_index_ - 1);
                        }
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Trajectory completed. All waypoints visited.");
                        // Optionally: initiate landing or hover
                    }

                    break;
                    

            }

			
		};
		timer_ = this->create_wall_timer(50ms, timer_callback);

    }

    // Define methods for drone control, such as taking off, landing, waypoint navigation, etc.
private:
    std::vector<std::array<float, 3>> square_waypoints_;
    bool armed_;
    void initializeWaypoints() {
        // Initialize waypoints forming a square trajectory at the specified takeoff height
        square_waypoints_ = {
            {0.0, 0.0, -15.0}, // Takeoff position (assuming NED coordinates)
            {50.0, 0.0, -15.0}, // First waypoint
            {50.0, 50.0, -15.0}, // Second waypoint
            {0.0, 50.0, -15.0} // Third waypoint
        };
  
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint(std::array<float, 3> position)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = position;
        msg.velocity = {NAN, NAN, NAN};
        msg.acceleration = {NAN, NAN, NAN};
        msg.yaw = NAN;
        msg.yawspeed = NAN;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }


    void vehicle_status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    }

    void arm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);

    }

    void disarm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);

    }

    void publish_vehicle_command(uint16_t command, float param1, float param2)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    FlightState get_current_state()
    {
        if (std::abs(drone_state.posZ) < kIsFlyingThreshold 
            && (current_state_ != FlightState::TAKING_OFF && current_state_ != FlightState::HOVERING
            && current_state_ != FlightState::FOLLOWING_TRAJECTORY))
        {
            return FlightState::WAITING;
        }
        else 
        {
            return current_state_;
        }
        // const bool landed = std::abs(drone_state.posZ) < 0.05;   // 5 cm
        //     if (!armed_ && landed && current_state_ != FlightState::TAKING_OFF) {
        //         return FlightState::WAITING;
        //     }
        //     return current_state_;
    }

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        
        float distance_squared =    pow(msg->x - next_waypoint[0], 2) +
                                    pow(msg->y - next_waypoint[1], 2) +
                                    pow(msg->z - next_waypoint[2], 2);
        drone_state.posX = msg->x;
        drone_state.posY = msg->y;
        drone_state.posZ = msg->z;

        waypoint_reached = distance_squared <= pow(waypoint_accuracy, 2);
        RCLCPP_INFO(this->get_logger(), "Curr location: %f, %f, %f", msg->x, msg->y, msg->z);
        if (waypoint_reached) 
        {
            RCLCPP_INFO(this->get_logger(), "Reached the vicinity of the desired waypoint.");

        } 
        else
        {
            RCLCPP_INFO(this->get_logger(), "Not yet within the vicinity of the desired waypoint.");
        }
    }

    double calculate_distance(const std::array<float, 3> start_waypoint, 
        const std::array<float, 3> end_waypoint) 
    {
        return std::hypot(end_waypoint[0] - start_waypoint[0], end_waypoint[1] - start_waypoint[1]
            ,end_waypoint[2] - start_waypoint[2]);
    }

    void generate_trajectory(int leg_distance)
    {
        trajectory_waypoints.push_back(square_waypoints_[0]);
        for(size_t i=0; i<(square_waypoints_.size()); i++)
        {
            auto start_waypoint = square_waypoints_[i];
            auto end_waypoint = square_waypoints_[i+1];
            if(i == (square_waypoints_.size()-1))
            {
                end_waypoint = square_waypoints_[0];
            }
            
            auto distance = calculate_distance(start_waypoint, end_waypoint);
            if (distance <= leg_distance) {
                trajectory_waypoints.push_back(end_waypoint);
            }
            int num_segments = std::ceil(distance/leg_distance);
            float dx = (end_waypoint[0] - start_waypoint[0])/num_segments;
            float dy = (end_waypoint[1] - start_waypoint[1])/num_segments;
            float dz = (end_waypoint[2] - start_waypoint[2])/num_segments;

             // Generate the intermediate waypoints.
            for (int j = 1; j <= num_segments; ++j) {
                trajectory_waypoints.push_back({start_waypoint[0] + j * dx, start_waypoint[1] + j * dy 
                    , start_waypoint[2] + j * dz});
                RCLCPP_INFO(this->get_logger(), "waypoint: %f, %f, %f", start_waypoint[0] + j * dx,start_waypoint[1] + j * dy,
                    start_waypoint[2] + j * dz);
            }
        }
    }
 
    uint64_t offboard_setpoint_counter_; 
    FlightState current_state_;
    std::vector<px4_msgs::msg::TrajectorySetpoint> waypoints_;
    uint64_t current_waypoint_index_;
    float waypoint_accuracy;
    std::array<float, 3> next_waypoint;
    bool waypoint_reached;
    DroneState drone_state;
    std::vector<std::array<float, 3>> trajectory_waypoints;
    
    float kIsFlyingThreshold;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        vehicle_status_sub_; 

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscriber;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Define member variables for publishers, subscribers, and other state variables
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
