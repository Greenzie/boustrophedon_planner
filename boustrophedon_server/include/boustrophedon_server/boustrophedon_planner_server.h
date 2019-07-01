#ifndef SRC_BOUSTROPHEDON_PLANNER_SERVER_H
#define SRC_BOUSTROPHEDON_PLANNER_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>

#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>

#include "cgal_utils.h"
#include "striping_planner.h"
#include "outline_planner.h"
#include "cellular_decomposition/polygon_decomposer.h"

class BoustrophedonPlannerServer
{
public:
  BoustrophedonPlannerServer();

  void executePlanPathAction(const boustrophedon_msgs::PlanMowingPathGoalConstPtr& goal);

private:
  using Server = actionlib::SimpleActionServer<boustrophedon_msgs::PlanMowingPathAction>;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  Server action_server_;
  ros::ServiceServer conversion_server_;

  StripingPlanner striping_planner_;
  OutlinePlanner outline_planner_;

  int outline_layer_count_{};
  double stripe_separation_{};
  double stripe_angle_{};
  tf::TransformListener transform_listener_{};

  bool convertStripingPlanToPath(boustrophedon_msgs::ConvertPlanToPath::Request& request,
                                 boustrophedon_msgs::ConvertPlanToPath::Response& response);
  boustrophedon_msgs::PlanMowingPathResult toResult(std::vector<NavPoint>&& path, const std::string& frame) const;
  Polygon fromBoundary(const geometry_msgs::PolygonStamped& boundary) const;
  Point fromPositionWithFrame(const geometry_msgs::PoseStamped& pose, const std::string& target_frame) const;
};

#endif  // SRC_BOUSTROPHEDON_PLANNER_SERVER_H
