#include "drone_control/path_planning_utils.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

namespace vtca::path_planner {
namespace {

using ::geometry_msgs::msg::Point;

}  // namespace

std::vector<PointOfInterest> DivideBoundingBoxByParts(
    const geometry_msgs::msg::Point& min_pt,
    const geometry_msgs::msg::Point& max_pt, int n_parts,
    std::shared_ptr<rclcpp::Clock> clock) {
  std::vector<PointOfInterest> pois;
  // TODO(mkedia): LOG something and return status ?
  if (n_parts <= 0) {
    return pois;
  }
  if (n_parts == 1) {
    pois.push_back(PointOfInterest(0, min_pt, max_pt, clock->now()));
    return pois;
  }
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
      Point min;
      Point max;
      min.x = min_pt.x + j * sub_width;
      min.y = min_pt.y + i * sub_height;
      max.x = min_pt.x + (j + 1) * sub_width;
      max.y = min_pt.y + (i + 1) * sub_height;
      pois.push_back(PointOfInterest(i * cols + j, min, max, clock->now()));
    }
  }
  return pois;
}

std::vector<PointOfInterest> DivideBoundingBoxByArea(
    const geometry_msgs::msg::Point& min_pt,
    const geometry_msgs::msg::Point& max_pt, double max_sub_area,
    std::shared_ptr<rclcpp::Clock> clock) {
  std::vector<PointOfInterest> pois;
  // TODO(mkedia): LOG something and return status ?
  if (max_sub_area <= 0) {
    return pois;
  }
  double x_length = max_pt.x - min_pt.x;
  double y_length = max_pt.y - min_pt.y;
  double total_area = x_length * y_length;

  if (total_area <= max_sub_area) {
    return {PointOfInterest(0, min_pt, max_pt, clock->now())};
  }

  int n_parts = static_cast<int>(ceil(total_area / max_sub_area));

  return DivideBoundingBoxByParts(min_pt, max_pt, n_parts, clock);
}

// TODO(mkedia): Following aspects can be improved
// - Handling aspect ratio based S curve (to minimize turns)
// - Handling missing survey area if swath width is not divisible by Y distance
// - Validation
// - Initial / Final poisition of Drone
geometry_msgs::msg::PoseArray ComputeWaypoints(
    const PointOfInterest& poi, std::shared_ptr<rclcpp::Clock> clock,
    double swath_width, double drone_altitude, double survey_altitude) {
  std::vector<std::tuple<double, double, double>> waypoints;
  // NOTE: Assumes drone starts at 0, 0, 0.
  waypoints.emplace_back(0.0, 0.0, 0.0);
  waypoints.emplace_back(0.0, 0.0, drone_altitude);
  double current_x = poi.min_pt_.x + swath_width / 2.0;
  double max_y = poi.max_pt_.y;
  double min_y = poi.min_pt_.y;
  waypoints.emplace_back(current_x, min_y, drone_altitude);

  int direction = 1;
  while (current_x < (poi.max_pt_.x - swath_width / 2.0)) {
    if (direction == 1) {
      waypoints.emplace_back(current_x, min_y, survey_altitude);
      waypoints.emplace_back(current_x, max_y, survey_altitude);
    } else {
      waypoints.emplace_back(current_x, max_y, survey_altitude);
      waypoints.emplace_back(current_x, min_y, survey_altitude);
    }
    current_x = current_x + swath_width;
    direction = -direction;
  }
  const std::tuple<double, double, double>& last_pt = waypoints.back();
  waypoints.emplace_back(std::get<0>(last_pt), std::get<1>(last_pt),
                         drone_altitude);
  waypoints.emplace_back(0.0, 0.0, 0.0);

  geometry_msgs::msg::PoseArray trajectory;
  trajectory.header.stamp = clock->now();
  trajectory.header.frame_id = "map";
  for (auto& [x, y, z] : waypoints) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    trajectory.poses.push_back(pose);
  }
  return trajectory;
}

}  // namespace vtca::path_planner
