#include <geographic_msgs/msg/geo_point.hpp>

#include "drone_control/src/path_planning_utils.hpp"
#include "drone_control/srv/plan_survey.hpp"
#include "drone_control/srv/planner_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vtca::path_planner {
namespace {

using drone_controller::srv::PlannerCommand;
}


// TODO(mkedia): Update or make this a function of altitude.
constexpr double kIrisSwathWidth = 10;
constexpr double kMaxAltitude = 1000;

// TODO(mkedia): Make it a map of drone to chargepods, or make it assignable to
// a drone.
constexpr double chargepod_x = 0;
constexpr double chargepod_y = 0;

enum class DroneState {
  AVAILABLE,   // Ready for a new task, at charging pod
  TRAVERSING,  // Flying to the assigned block
  SURVEYING,   // Executing the lawn mower pattern over the block
  RETURNING,   // Flying back to the charging pod
  CHARGING,    // Out of service, charging
  LOST         // Lost communication beacon
};

// Stores Drone status updates as per Data published by drones.
struct Drone {
  int id;
  DroneState state = DroneState::CHARGING;
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Point pod_position;
  double altitude;
  double task_timer = 0.0;
  double charge_timer = 0.0;
  rclcpp::Time last_heartbeat;
  double speed;

  // Define constructor if needed.
};

class PathPlanner : public rclcpp::Node {
 public:
  PathPlanner() : Node('path_planner') {
    // TODO(mkedia): Some global param for frame of reference of Coordinates.

    command_subscriber_ = this->create_subscription<PlannerCommand>(
        "/planner/command", 10,
        std::bind(&PathPlanner::CommandCallback&, this, std::placeholders::_1));

    drone_subsrciber_ = this->create_subscription<DroneUpdate>(
        "/drone/update", 10,
        std::bind(&PathPlanner::DroneUpdateCb&, this, std::placeholders::1_));
  }

 private:
  void DroneUpdateCb(const DroneUpdate::SharedPtr msg) {
    // Update Drone state in drone map.
  }

  void CommandCallback(const PlannerCommand::SharedPtr msg) {
    // Validate if the message is valid.
    ValidatePlannerCommand(msg);
    // Compute pre-requisites for the plan & log.
    ComputePlan(msg);
    if (!msg->log_only) {
      CleanupExistingStateAndResetDrones();
      StartNewSurvey();
    }
  }

  void ValidatePlannerCommand(
      const drone_controller::srv::PlannerCommand::SharedPtr msg) {}

  void ComputePlan(const drone_controller::srv::PlannerCommand::SharedPtr msg) {
    // ComputeSurveyBlocks(msg);
    geographic_msgs::msg::BoundingBox bounding_box;
    bounding_box.min_pt.latitude = msg->latitude_min;
    bounding_box.min_pt.longitude = msg->longitude_min;
    bounding_box.max_pt.latitude = msg->latitude_max;
    bounding_box.max_pt.longitude = msg->longitude_max;

    // TODO(mkedia): Default to some origin.
    geographic_msgs::msg::GeoPoint map_origin;
    map_origin.latitude = msg->origin_latitude;
    map_origin.longitude = msg->origin_longitude;
    map_origin.altitude = msg->origin_altitude;

    geodesy::UTMPoint utm_origin(map_origin);

    geodesy::UTMPoint utm_min_pt(bounding_box.min_pt);
    geodesy::UTMPoint utm_max_pt(bounding_box.max_pt);

    Point2D local_min_pt = {utm_min_pt.easting - utm_origin.easting,
                            utm_min_pt.northing - utm_origin.northing};
    Point2D local_max_pt = {utm_max_pt.easting - utm_origin.easting,
                            utm_max_pt.northing - utm_origin.northing};

    RCLCPP_INFO(this->get_logger(), "Local Bounding Box Min (x, y): %f, %f",
                local_min_pt.x, local_min_pt.y);
    RCLCPP_INFO(this->get_logger(), "Local Bounding Box Max (x, y): %f, %f",
                local_max_pt.x, local_max_pt.y);

    std::vector<PointOfInterest> pois = DivideBoundingBoxByArea()

    // compute area
    // compute area per drone
    // divide area into blocks and initialize blocks

    // Compute PointsOfInterest & everything needed to start the survey.
    // Log plans.
    // Drone block size
  }

  // Forward declaration of the tiling function
  std::vector<PointOfInterest> DivideBoundingBoxByParts(const Point2D& min_pt,
                                                        const Point2D& max_pt,
                                                        int n_parts);

  std::vector<PointOfInterest> DivideBoundingBoxByArea(const Point2D& min_pt,
                                                       const Point2D& max_pt,
                                                       double max_sub_area) {
    if (max_sub_area <= 0) {
      throw std::invalid_argument("Maximum sub-box area must be positive.");
    }

    // TODO(mkedia): Do mod ?
    double x_length = max_pt.x - min_pt.x;
    double y_length = max_pt.y - min_pt.y;
    double total_area = x_length * y_length;

    // If the box is already smaller than the max allowed area, no division is
    // needed.
    if (total_area <= max_sub_area) {
      return {main_box};  // Return a vector containing only the original box
    }

    // Calculate how many pieces are needed to satisfy the max area constraint.
    // We use ceil to ensure the resulting sub-box area is not larger than
    // max_sub_area.
    int n_parts = static_cast<int>(ceil(total_area / max_sub_area));

    // === Step 2: Use the robust tiling algorithm to divide the area into N
    // parts ===
    return DivideBoundingBoxByParts(min_pt, max_pt, n_parts);
  }

  std::vector<PointOfInterest> DivideBoundingBoxByParts(const Point2D& min_pt,
                                                        const Point2D& max_pt,
                                                        int n_parts) {
    if (n_parts <= 0) {
      throw std::invalid_argument(
          "Number of parts must be a positive integer.");
    }

    std::vector<PointOfInterest> sub_boxes;
    if (n_parts == 1) {
      // TODO(add constructor with index, min & max pt to PointOfInterest.
      sub_boxes.push_back(PointOfInterest(0, min_pt, max_pt));
      return sub_boxes;
    }

    // Find two factors of n_parts that are closest to its square root
    int rows = 0;
    int cols = 0;
    int best_factor = 1;
    for (int i = 1; i * i <= n_parts; ++i) {
      if (n_parts % i == 0) {
        best_factor = i;
      }
    }

    double aspect_ratio = (max_pt.x - min_pt.x) / (max_pt.y - min_pt.y);
    if (aspect_ratio >= 1.0) {  // Wider than tall
      cols = n_parts / best_factor;
      rows = best_factor;
    } else {  // Taller than wide
      cols = best_factor;
      rows = n_parts / best_factor;
    }

    double x_length = max_pt.x - min_pt.x;
    double y_length = max_pt.y - min_pt.y;
    double sub_width = x_length / cols;
    double sub_height = y_length / rows;

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        Point2D min;
        Point2D max;
        min.x = min_pt.x + j * sub_width;
        min.y = min_pt.y + i * sub_height;
        max.x = min_pt.x + (j + 1) * sub_width;
        max.y = min_pt.y + (i + 1) * sub_height;
        sub_boxes.push_back(PointOfInterest(i * cols + j, min, max));
      }
    }
    return sub_boxes;
  }

  void CleanupExistingStateAndResetDrones() {
    // Clean up any state variables.
    //  - Not the drone map <since that one is computed from the last heartbeats
    //  of all the drones>
    // Publish topics to Drones to standby
    // Reset existing planner_timer to nullptr.
  }

  void StartNewSurvey() {
    auto timer_period = std::chrono::duration<double>(1.0 / update_frequency_);
    planner_timer_ = this->create_wall_timer(
        timer_period, std::bind(&PathPlannerNode::PlannerControlFlow, this));
  }

  void PlannerControlFlow() {
    rclcpp::Time current_time = this->now();
    CheckDrones();
    UpdateRewards();
    AssignTasks();
  }

  void CheckDrones() {}

  void UpdateRewards() {}

  void AssignTasks() {}

  void planSurveyCb(
      const std::shared_ptr<drone_control::srv::PlanSurvey::Request> request,
      std::shared_ptr<drone_control::srv::PlanSurvey::Response> response) {
    double min_x = request->min_x;
    double min_y = request->min_y;
    double max_x = request->max_x;
    double max_y = request->max_y;
    int num_drones = request->num_drones;

    // TODO(mkedia): Validate request.

    // NOTE(mkedia): Assume surplus capacity for now (min twice as much or more
    // to enable continuous surveillance including charge time) NOTE(mkedia):
    // Assume charging time is less than survey time ?!

    // Drone velocity
    // Num drones - 5
    // Compute Total drone time  - 100

    // Divide by battery capacity / drone - 30
    // Assign altitude to drone
    // CEIL(Total drone time x 2 /  Time per drone) = num parts of rectangle
    // Assign rectangle to drone as waypoints + intelligently route drone from &
    // to charging point
  }

  std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>
      drone_waypoint_publishers_;
  rclcpp::Service<drone_control::srv::PlanSurvey>::SharedPtr
      plan_survey_service_;
  rclcpp::TimerBase::SharedPtr planner_timer_;
  std::hash_map<int32, Drone> available_drones_;
};

}  // namespace vtca::path_planner
