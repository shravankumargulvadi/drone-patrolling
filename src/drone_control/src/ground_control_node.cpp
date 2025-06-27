#include <cmath>
#include <tuple>
#include <vector>

#include "drone_control/msg/drone_status.hpp"
#include "drone_control/path_planning_utils.hpp"
#include "drone_control/srv/path_planner.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

namespace {

using drone_control::msg::DroneStatus;
using drone_control::srv::PathPlanner;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseArray;
using vtca::path_planner::DivideBoundingBoxByArea;
using vtca::path_planner::PointOfInterest;

constexpr double kX500SwathWidth = 10;
constexpr double kSurveyAltitude = -20.0;
constexpr double kMaxAltitude = 1000;
constexpr double kDroneSurveyCapacity = 10000;
static const rclcpp::Duration kMaxAssignedDuration =
    rclcpp::Duration::from_seconds(60 * 60);  // 1 hour.
}  // namespace

class GCSNode : public rclcpp::Node {
 public:
  GCSNode() : Node("gcs_node") {
    this->declare_parameter<std::vector<std::string>>("drone_ids", {""});
    this->get_parameter("drone_ids", drone_ids_);

    service_ = this->create_service<drone_control::srv::PathPlanner>(
        "path_planner",
        std::bind(&GCSNode::PathPlannerService, this, std::placeholders::_1,
                  std::placeholders::_2));

    int index = 1;
    for (const auto& id : drone_ids_) {
      auto sub = this->create_subscription<DroneStatus>(
          "/" + id + "/drone/update", 10,
          [this, id](const DroneStatus::SharedPtr msg) {
            this->DroneUpdateSubscriberCb(msg, id);
          });
      drone_subscribers_.push_back(sub);

      auto pub = this->create_publisher<PoseArray>(
          "/" + id + "/trajectory_upload", 10);
      trajectory_publishers_.insert({id, pub});

      // Assign drone altitudes "slots" above survey altitude.
      drone_altitudes_.insert({id, kSurveyAltitude - (index * 2)});
      index++;
    }

    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&GCSNode::SurveyStandby, this));
    RCLCPP_INFO(this->get_logger(), "Path planning service is ready.");
  }

 private:
  rclcpp::Service<PathPlanner>::SharedPtr service_;
  std::unordered_map<
      std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr>
      trajectory_publishers_;
  std::vector<rclcpp::Subscription<DroneStatus>::SharedPtr> drone_subscribers_;
  // Allocated altitude to drones for traveling to & from surveys.
  std::unordered_map<std::string, double> drone_altitudes_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> drone_ids_;
  std::vector<PointOfInterest> pois_;
  std::mutex poi_mutex_;
  bool survey_on_ = false;

  void PathPlannerService(const std::shared_ptr<PathPlanner::Request> request,
                          std::shared_ptr<PathPlanner::Response> response) {
    RCLCPP_INFO(this->get_logger(),
                "Incoming request:\n"
                "  min_x: %.6f, min_y: %.6f\n"
                "  max_x: %.6f, max_y: %.6f\n",
                request->min_x, request->min_y, request->max_x, request->max_y);
    ValidatePlannerCommand(request);
    ConstructPointsOfInterests(request);
    if (StartSurvey()) {
      response->goal_status.status =
          action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
    } else {
      response->goal_status.status =
          action_msgs::msg::GoalStatus::STATUS_ABORTED;
      RCLCPP_WARN(this->get_logger(),
                  "No Points of interests were generated, aborting survey.");
    }
  }

  // Wrap up previous survey if any ?
  void ValidatePlannerCommand(
      const std::shared_ptr<PathPlanner::Request> /*request*/) {
    // TODO(mkedia): Validate request.
  }

  void ConstructPointsOfInterests(
      const std::shared_ptr<PathPlanner::Request> request) {
    Point local_min_pt;
    local_min_pt.x = request->min_x;
    local_min_pt.y = request->min_y;
    local_min_pt.z = kSurveyAltitude;
    Point local_max_pt;
    local_max_pt.x = request->max_x;
    local_max_pt.y = request->max_y;
    local_max_pt.z = kSurveyAltitude;

    RCLCPP_INFO(this->get_logger(), "Local Bounding Box Min (x, y): %f, %f",
                local_min_pt.x, local_min_pt.y);
    RCLCPP_INFO(this->get_logger(), "Local Bounding Box Max (x, y): %f, %f",
                local_max_pt.x, local_max_pt.y);

    // Acquire lock to modify POIs.
    {
      std::lock_guard<std::mutex> lock(poi_mutex_);
      pois_ = DivideBoundingBoxByArea(local_min_pt, local_max_pt,
                                      kDroneSurveyCapacity, this->get_clock());
    }
  }

  bool StartSurvey() {
    if (!pois_.empty()) {
      survey_on_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Starting new survey with %d drones and %d number of survey "
                  "blocks for the entire survey area.",
                  drone_ids_.size(), pois_.size());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Failed to start new survey with %d drones. No survey blocks "
                  "generated.",
                  drone_ids_.size());
    }
    return survey_on_;
  }

  void SurveyStandby() {
    if (!survey_on_) {
      return;
    }
    // Acquire lock to modify POIs.
    std::lock_guard<std::mutex> lock(poi_mutex_);
    for (auto& poi : pois_) {
      double since_last_survey_secs =
          (this->get_clock()->now() - poi.last_completed_survey_time_)
              .seconds();
      poi.reward_ = static_cast<int32_t>(since_last_survey_secs / 60);
      if (poi.assignment_ == PointOfInterest::SurveyAssignment::ASSIGNED &&
          poi.last_assigned_time_.nanoseconds() != 0 &&
          (this->get_clock()->now() - poi.last_assigned_time_) >
              kMaxAssignedDuration) {
        poi.allocated_drone_id_ = "";
        poi.last_assigned_time_ = rclcpp::Time(0, 0);
        poi.assignment_ = PointOfInterest::SurveyAssignment::UNASSIGNED;
      }
    }
  }

  void SendTrajectory(const std::string& drone_id, const PointOfInterest& poi) {
    RCLCPP_INFO(this->get_logger(), "Sending trajectory for  poi for drone: %s",
                drone_id.c_str());
    if (auto drone_altitude = drone_altitudes_.find(drone_id);
        drone_altitude != drone_altitudes_.end()) {
      RCLCPP_INFO(this->get_logger(), "Computing waypoints for drone: %s",
                  drone_id.c_str());
      geometry_msgs::msg::PoseArray trajectory =
          vtca::path_planner::ComputeWaypoints(
              poi, this->get_clock(), kX500SwathWidth, drone_altitude->second,
              kSurveyAltitude);
      RCLCPP_INFO(this->get_logger(), "Sending waypoints for drone: %s",
                  drone_id.c_str());
      trajectory_publishers_[drone_id]->publish(trajectory);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Did not send trajectory to Drone id: %s because couldn't "
                  "find drone specific altitude.",
                  drone_id);
      // TODO(mkedia): Any other error handling.
    }
  }

  void DroneUpdateSubscriberCb(const DroneStatus::SharedPtr& update,
                               const std::string& drone_id) {
    std::lock_guard<std::mutex> lock(poi_mutex_);
    if (update->status == DroneStatus::STATE_AVAILABLE) {
      PointOfInterest* max_reward_poi = nullptr;
      int32_t max_reward = -1;
      for (auto& poi : pois_) {
        if (poi.allocated_drone_id_.empty() &&
            poi.assignment_ == PointOfInterest::SurveyAssignment::UNASSIGNED) {
          if (poi.reward_ > max_reward) {
            max_reward_poi = &poi;
            max_reward = poi.reward_;
          }
        }
      }
      if (max_reward_poi != nullptr) {
        max_reward_poi->allocated_drone_id_ = drone_id;
        max_reward_poi->assignment_ =
            PointOfInterest::SurveyAssignment::ASSIGNED;
        max_reward_poi->last_assigned_time_ = this->get_clock()->now();
      }
      if (max_reward_poi != nullptr) {
        RCLCPP_INFO(this->get_logger(),
                    "Sending trajectory for max reward poi for drone: %s",
                    drone_id.c_str());
        SendTrajectory(drone_id, *max_reward_poi);
      }
    }
    if (update->status == DroneStatus::STATE_STARTING_SURVEY) {
      PointOfInterest* allocated_poi = nullptr;
      for (auto& poi : pois_) {
        if (poi.allocated_drone_id_ == drone_id) {
          allocated_poi = &poi;
          break;
        }
      }
      // TODO(mkedia): Handle case where assignment_ isn't ASSIGNED.
      if (allocated_poi != nullptr) {
        allocated_poi->assignment_ =
            PointOfInterest::SurveyAssignment::CONFIRMED;
      }
    }
    if (update->status == DroneStatus::STATE_SURVEY_COMPLETE) {
      PointOfInterest* allocated_poi = nullptr;
      for (auto& poi : pois_) {
        if (poi.allocated_drone_id_ == drone_id) {
          allocated_poi = &poi;
          break;
        }
      }
      // TODO(mkedia): Handle case where assignment_ isn't CONFIRMED
      // meaning, that the STATE_STARTING_SURVEY update wasn't received.
      if (allocated_poi != nullptr) {
        allocated_poi->reward_ = 0;
        allocated_poi->allocated_drone_id_ = "";
        allocated_poi->last_completed_survey_time_ = this->get_clock()->now();
        allocated_poi->assignment_ =
            PointOfInterest::SurveyAssignment::UNASSIGNED;
      } else {
        RCLCPP_WARN(
            this->get_logger(),
            "Did not find POI which was previously assigned drone_id: %s, even "
            "though drone update suggests survey was complete",
            drone_id.c_str());
      }
    }
    if (update->status == DroneStatus::STATE_SURVEY_ONGOING) {
      RCLCPP_INFO(this->get_logger(), "Drone %s is surveying",
                  drone_id.c_str());
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GCSNode>();
  auto client = node->create_client<PathPlanner>("path_planner");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto request = std::make_shared<PathPlanner::Request>();
  request->min_x = 10.0;
  request->min_y = 10.0;
  request->max_x = 1010.0;
  request->max_y = 1010.0;

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Survey accepted");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service planner_service");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
