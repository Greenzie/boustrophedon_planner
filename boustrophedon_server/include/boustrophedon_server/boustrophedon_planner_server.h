#ifndef SRC_BOUSTROPHEDON_PLANNER_SERVER_H
#define SRC_BOUSTROPHEDON_PLANNER_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>
#include <nav_msgs/Odometry.h>

#include "boustrophedon_server/cgal_utils.h"
#include "boustrophedon_server/striping_planner.h"
#include "boustrophedon_server/outline_planner.h"
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
  ros::Publisher initial_polygon_publisher_;
  ros::Publisher preprocessed_polygon_publisher_;
  ros::Publisher path_points_publisher_;
  ros::Publisher polygon_points_publisher_;

  StripingPlanner striping_planner_;
  OutlinePlanner outline_planner_;

  bool repeat_boundary_{};
  bool outline_clockwise_{};
  bool skip_outlines_{};
  bool enable_orientation_{};
  int outline_layer_count_{};
  double stripe_separation_{};
  double intermediary_separation_{};
  double stripe_angle_{};
  bool travel_along_boundary_{};
  bool allow_points_outside_boundary_{};
  bool enable_half_y_turns_{};
  int points_per_turn_{};
  double turn_start_offset_{};
  tf::TransformListener transform_listener_{};
  bool publish_polygons_{};
  bool publish_path_points_{};

  bool convertStripingPlanToPath(boustrophedon_msgs::ConvertPlanToPath::Request& request,
                                 boustrophedon_msgs::ConvertPlanToPath::Response& response);
  boustrophedon_msgs::PlanMowingPathResult toResult(std::vector<NavPoint>&& path, const std::string& frame) const;
  Polygon fromBoundary(const geometry_msgs::PolygonStamped& boundary) const;
  Point fromPositionWithFrame(const geometry_msgs::PoseStamped& pose, const std::string& target_frame) const;
  bool checkPolygonIsValid(const Polygon& poly) const;
  double getStripeAngleFromOrientation(const geometry_msgs::PoseStamped& robot_position);
  geometry_msgs::PolygonStamped convertCGALPolygonToMsg(const Polygon& poly) const;
  void publishPathPoints(const std::vector<NavPoint>& path) const;
  void publishPolygonPoints(const Polygon& poly) const;
};

#endif  // SRC_BOUSTROPHEDON_PLANNER_SERVER_H
