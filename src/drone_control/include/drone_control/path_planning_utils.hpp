#ifndef DRONE_CONTROL__PATH_PLANNING_UTILS_HPP_
#define DRONE_CONTROL__PATH_PLANNING_UTILS_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

namespace vtca::path_planner {

// Object represents a specific PointOfInterest.
// To be fleshed out into a Node of a SurveyArea Graph.
class PointOfInterest {
 public:
  PointOfInterest(const std::string& name,
                  const geometry_msgs::msg::Point& min_pt,
                  const geometry_msgs::msg::Point& max_pt,
                  rclcpp::Time last_completed_survey_time)
      : name_(name),
        min_pt_(min_pt),
        max_pt_(max_pt),
        last_completed_survey_time_(last_completed_survey_time) {}

  PointOfInterest(const int32_t index, const geometry_msgs::msg::Point& min_pt,
                  const geometry_msgs::msg::Point& max_pt,
                  rclcpp::Time last_completed_survey_time)
      : name_("poi_" + std::to_string(index)),
        min_pt_(min_pt),
        max_pt_(max_pt),
        last_completed_survey_time_(last_completed_survey_time) {}

  PointOfInterest(const PointOfInterest& other)
      : name_(other.name_),
        min_pt_(other.min_pt_),
        max_pt_(other.max_pt_),
        reward_(other.reward_),
        assignment_(other.assignment_),
        last_assigned_time_(other.last_assigned_time_),
        allocated_drone_id_(other.allocated_drone_id_),
        last_completed_survey_time_(other.last_completed_survey_time_) {}

  enum class SurveyAssignment { UNASSIGNED, ASSIGNED, CONFIRMED };

  std::string name_;
  ::geometry_msgs::msg::Point min_pt_;
  ::geometry_msgs::msg::Point max_pt_;
  int32_t reward_ = 0;

  SurveyAssignment assignment_ = SurveyAssignment::UNASSIGNED;
  rclcpp::Time last_assigned_time_;

  std::string allocated_drone_id_;
  rclcpp::Time last_completed_survey_time_;

 private:
  // TODO(mkedia): move variables to private and provide getter / setter.
};

std::vector<PointOfInterest> DivideBoundingBoxByParts(
    const geometry_msgs::msg::Point& min_pt,
    const geometry_msgs::msg::Point& max_pt, int n_parts,
    std::shared_ptr<rclcpp::Clock> clock);

std::vector<PointOfInterest> DivideBoundingBoxByArea(
    const geometry_msgs::msg::Point& min_pt,
    const geometry_msgs::msg::Point& max_pt, double max_sub_area,
    std::shared_ptr<rclcpp::Clock> clock);

geometry_msgs::msg::PoseArray ComputeWaypoints(
    const PointOfInterest& poi, std::shared_ptr<rclcpp::Clock> clock,
    double swath_width, double drone_altitude, double survey_altitude);

}  // namespace vtca::path_planner

#endif DRONE_CONTROL__PATH_PLANNING_UTILS_HPP_
