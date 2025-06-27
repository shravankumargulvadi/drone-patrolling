#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "drone_control/msg/drone_status.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
using namespace std::chrono_literals;
using drone_control::msg::DroneStatus;
}  // namespace

enum class FlightState {
  WAITING,
  TAKING_OFF,
  HOVERING,
  FOLLOWING_TRAJECTORY,
  COMPLETED
};

class DroneManager : public rclcpp::Node {
 public:
  DroneManager() : Node("drone_manager") {
    this->declare_parameter<int>("drone_id", 0);
    this->get_parameter("drone_id", drone_id_);

    // TODO(mkedia): Drone id for ground station is different.
    std::string ns = "/px4_" + std::to_string(drone_id_);

    std::string offboard_topic = ns + "/fmu/in/offboard_control_mode";
    std::string trajectory_topic = ns + "/fmu/in/trajectory_setpoint";
    std::string vehicle_cmd_topic = ns + "/fmu/in/vehicle_command";
    std::string status_topic = ns + "/fmu/out/vehicle_status";
    std::string local_pos_topic = ns + "/fmu/out/vehicle_local_position";
    std::string traj_upload_topic = ns + "/trajectory_upload";
    std::string status_update_topic = ns + "/drone/update";

    RCLCPP_INFO(this->get_logger(), "Publishing to:");
    RCLCPP_INFO(this->get_logger(), "  %s", offboard_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  %s", trajectory_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  %s", vehicle_cmd_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  %s", status_update_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to:");
    RCLCPP_INFO(this->get_logger(), "  %s", status_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  %s", local_pos_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  %s", traj_upload_topic.c_str());

    offboard_control_mode_pub_ =
        this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            offboard_topic, 10);
    trajectory_setpoint_pub_ =
        this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            trajectory_topic, 10);
    vehicle_command_pub_ =
        this->create_publisher<px4_msgs::msg::VehicleCommand>(
            vehicle_cmd_topic,
            /*qos_history_depth=*/10);
    status_pub_ = this->create_publisher<DroneStatus>(status_update_topic,
                                                      /*qos_history_depth=*/10);

    rclcpp::QoS qos_profile{rclcpp::SensorDataQoS()};

    vehicle_status_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleStatus>(
            status_topic, qos_profile,
            std::bind(&DroneManager::vehicle_status_callback, this,
                      std::placeholders::_1));

    local_position_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            local_pos_topic, qos_profile,
            std::bind(&DroneManager::local_position_callback, this,
                      std::placeholders::_1));

    trajectory_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        traj_upload_topic, 10,
        std::bind(&DroneManager::trajectory_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        50ms, std::bind(&DroneManager::control_loop, this));
  }

  // Mirroring the state in DroneStatus.
  enum class DroneStatusEnum {
    STATE_AVAILABLE = DroneStatus::STATE_AVAILABLE,
    STATE_STARTING_SURVEY = DroneStatus::STATE_STARTING_SURVEY,
    STATE_SURVEY_ONGOING = DroneStatus::STATE_SURVEY_ONGOING,
    STATE_SURVEY_COMPLETE = DroneStatus::STATE_SURVEY_COMPLETE
  };

 private:
  int drone_id_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      vehicle_command_pub_;
  rclcpp::Publisher<DroneStatus>::SharedPtr status_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      trajectory_sub_;

  FlightState current_state_ = FlightState::WAITING;
  std::vector<std::array<float, 3>> waypoints_;
  uint64_t waypoint_index_ = 0;
  bool armed_ = false;
  bool waypoint_reached_ = false;
  uint64_t offboard_setpoint_counter_ = 0;
  std::array<float, 3> next_waypoint_ = {0.0, 0.0, -8.0};

  float posX_, posY_, posZ_;
  const float waypoint_accuracy_ = 0.5;

  void PublishDroneStatus(DroneStatusEnum status) {
    DroneStatus update;
    update.status = static_cast<uint8_t>(status);
    geometry_msgs::msg::Point point;
    point.x = posX_;
    point.y = posY_;
    point.z = posZ_;
    update.position = point;
    status_pub_->publish(update);
  }

  void trajectory_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    waypoints_.clear();
    for (const auto& pose : msg->poses) {
      std::array<float, 3> waypoint = {static_cast<float>(pose.position.x),
                                       static_cast<float>(pose.position.y),
                                       static_cast<float>(pose.position.z)};
      waypoints_.push_back(waypoint);
    }
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu waypoints",
                waypoints_.size());
    current_state_ = FlightState::TAKING_OFF;
    waypoint_index_ = 0;
    PublishDroneStatus(DroneStatusEnum::STATE_STARTING_SURVEY);
  }

  void publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.position = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint(const std::array<float, 3>& position) {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position = position;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = drone_id_ + 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
  }

  void arm() {
    publish_vehicle_command(400, 1.0f, 0.0f);
  }  // VEHICLE_CMD_COMPONENT_ARM_DISARM
  void set_offboard_mode() {
    publish_vehicle_command(176, 1.0f, 6.0f);
  }  // VEHICLE_CMD_DO_SET_MODE

  void vehicle_status_callback(
      const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    armed_ = (msg->arming_state == msg->ARMING_STATE_ARMED);
  }

  void local_position_callback(
      const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    posX_ = msg->x;
    posY_ = msg->y;
    posZ_ = msg->z;
    float dx = posX_ - next_waypoint_[0];
    float dy = posY_ - next_waypoint_[1];
    float dz = posZ_ - next_waypoint_[2];
    waypoint_reached_ = (dx * dx + dy * dy + dz * dz) <
                        (waypoint_accuracy_ * waypoint_accuracy_);
  }

  void control_loop() {
    publish_offboard_control_mode();

    switch (current_state_) {
      case FlightState::WAITING:
        RCLCPP_INFO_ONCE(this->get_logger(), "State: WAITING");
        PublishDroneStatus(DroneStatusEnum::STATE_AVAILABLE);
        break;

      case FlightState::TAKING_OFF:
        PublishDroneStatus(DroneStatusEnum::STATE_STARTING_SURVEY);
        RCLCPP_INFO(this->get_logger(), "State: TAKING_OFF");
        publish_trajectory_setpoint(next_waypoint_);
        if (++offboard_setpoint_counter_ == 10) {
          set_offboard_mode();
          arm();
        }

        if (waypoint_reached_) {
          current_state_ = FlightState::HOVERING;
          offboard_setpoint_counter_ = 0;
        }
        break;

      case FlightState::HOVERING:
        RCLCPP_INFO_ONCE(this->get_logger(), "State: HOVERING");
        current_state_ = FlightState::FOLLOWING_TRAJECTORY;
        break;

      case FlightState::FOLLOWING_TRAJECTORY:
        RCLCPP_INFO_ONCE(this->get_logger(), "State: FOLLOWING_TRAJECTORY");
        if (waypoint_index_ < waypoints_.size()) {
          next_waypoint_ = waypoints_[waypoint_index_];
          PublishDroneStatus(DroneStatusEnum::STATE_SURVEY_ONGOING);
          publish_trajectory_setpoint(next_waypoint_);
          if (waypoint_reached_) {
            RCLCPP_INFO(this->get_logger(),
                        "Reached waypoint %lu: (%.2f, %.2f, %.2f)",
                        waypoint_index_, next_waypoint_[0], next_waypoint_[1],
                        next_waypoint_[2]);

            ++waypoint_index_;
          }
        } else {
          current_state_ = FlightState::COMPLETED;
        }
        break;
      case FlightState::COMPLETED:
        RCLCPP_INFO(this->get_logger(), "State: COMPLETED");
        PublishDroneStatus(DroneStatusEnum::STATE_SURVEY_COMPLETE);
        current_state_ = FlightState::WAITING;
        break;
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneManager>());
  rclcpp::shutdown();
  return 0;
}
