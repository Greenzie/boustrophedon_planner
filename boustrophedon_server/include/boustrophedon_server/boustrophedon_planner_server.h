#ifndef SRC_BOUSTROPHEDON_PLANNER_SERVER_H
#define SRC_BOUSTROPHEDON_PLANNER_SERVER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "boustrophedon_msgs/action/plan_mowing_path.hpp"
#include "boustrophedon_msgs/srv/convert_plan_to_path.hpp"
#include "boustrophedon_server/cgal_utils.h"
#include "boustrophedon_server/striping_planner.h"
#include "boustrophedon_server/outline_planner.h"
#include "cellular_decomposition/polygon_decomposer.h"

class BoustrophedonPlannerServer : public rclcpp::Node
{
public:
  BoustrophedonPlannerServer();

  using PlanMowingPathAction = boustrophedon_msgs::action::PlanMowingPath;
  using GoalHandlePlanMowingPathAction = rclcpp_action::ServerGoalHandle<PlanMowingPathAction>;
  using StripingPointMsg = boustrophedon_msgs::msg::StripingPoint;
  using ConvertPlanToPathSrv = boustrophedon_msgs::srv::ConvertPlanToPath;

private:
  rclcpp_action::Server<PlanMowingPathAction>::SharedPtr action_server_;

  rclcpp::Service<ConvertPlanToPathSrv>::SharedPtr conversion_server_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr initial_polygon_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr preprocessed_polygon_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr path_points_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr polygon_points_publisher_;

  StripingPlanner striping_planner_;
  OutlinePlanner outline_planner_;

  bool repeat_boundary_;
  bool outline_clockwise_;
  bool skip_outlines_;
  int outline_layer_count_;
  double stripe_separation_;
  double intermediary_separation_;
  double stripe_angle_;
  bool stripe_angle_from_robot_orientation;
  bool stripe_angle_from_boundary_orientation;
  bool travel_along_boundary_;
  bool allow_points_outside_boundary_;
  bool enable_half_y_turns_;
  int points_per_turn_;
  double turn_start_offset_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  bool publish_polygons_;
  bool publish_path_points_;

  void convertStripingPlanToPath(ConvertPlanToPathSrv::Request::SharedPtr request,
                                 ConvertPlanToPathSrv::Response::SharedPtr response);
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PlanMowingPathAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePlanMowingPathAction> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePlanMowingPathAction> goal_handle);
  void executePlanPathAction(const std::shared_ptr<GoalHandlePlanMowingPathAction> goal_handle);
  PlanMowingPathAction::Result toResult(std::vector<NavPoint>&& path, const std::string& frame) const;
  Polygon fromBoundary(const geometry_msgs::msg::PolygonStamped& boundary) const;
  Point fromPositionWithFrame(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame) const;
  bool checkPolygonIsValid(const Polygon& poly) const;
  double getStripeAngleFromOrientation(const geometry_msgs::msg::PoseStamped& robot_position);
  double getPolygonOrientation(const Polygon& poly);
  geometry_msgs::msg::PolygonStamped convertCGALPolygonToMsg(const Polygon& poly) const;
  void publishPathPoints(const std::vector<NavPoint>& path);
  void publishPolygonPoints(const Polygon& poly);
};

#endif  // SRC_BOUSTROPHEDON_PLANNER_SERVER_H
