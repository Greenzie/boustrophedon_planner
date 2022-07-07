#ifndef SRC_BOUSTROPHEDON_PLANNER_CLIENT_H
#define SRC_BOUSTROPHEDON_PLANNER_CLIENT_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <boustrophedon_msgs/action/plan_mowing_path.hpp>
#include <boustrophedon_msgs/srv/convert_plan_to_path.hpp>
#include <nav_msgs/msg/odometry.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"


class BoustrophedonPlannerClient : public rclcpp::Node
{
public:
  BoustrophedonPlannerClient();

  using PlanMowingPathAction = boustrophedon_msgs::action::PlanMowingPath;
  using GoalHandlePlanMowingPathAction = rclcpp_action::ClientGoalHandle<PlanMowingPathAction>;
  using StripingPointMsg = boustrophedon_msgs::msg::StripingPoint;
  using ConvertPlanToPathSrv = boustrophedon_msgs::srv::ConvertPlanToPath;

private:
  rclcpp_action::Client<PlanMowingPathAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string boundary_file;

  void send_goal();
  void goal_response_callback(std::shared_future<GoalHandlePlanMowingPathAction::SharedPtr> future);
  void feedback_callback(GoalHandlePlanMowingPathAction::SharedPtr, const std::shared_ptr<const PlanMowingPathAction::Feedback> feedback);
  void result_callback(const GoalHandlePlanMowingPathAction::WrappedResult & result);
};

#endif  // SRC_BOUSTROPHEDON_PLANNER_CLIENT_H
