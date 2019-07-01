#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <boustrophedon_server/boustrophedon_planner_server.h>

BoustrophedonPlannerServer::BoustrophedonPlannerServer()
  : private_node_handle_("~")
  , action_server_(node_handle_, "plan_path", boost::bind(&BoustrophedonPlannerServer::executePlanPathAction, this, _1),
                   false)
  , conversion_server_{ node_handle_.advertiseService("convert_striping_plan_to_path",
                                                      &BoustrophedonPlannerServer::convertStripingPlanToPath, this) }
{
  std::size_t error = 0;
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_layer_count", outline_layer_count_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_separation", stripe_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_angle", stripe_angle_));
  rosparam_shortcuts::shutdownIfError("plan_path", error);

  striping_planner_.setParameters({ stripe_separation_ });
  outline_planner_.setParameters({ outline_layer_count_, stripe_separation_ });

  action_server_.start();
}

void BoustrophedonPlannerServer::executePlanPathAction(const boustrophedon_msgs::PlanMowingPathGoalConstPtr& goal)
{
  std::string boundary_frame = goal->property.header.frame_id;
  Polygon polygon = fromBoundary(goal->property);

  Point robot_position;
  try
  {
    robot_position = fromPositionWithFrame(goal->robot_position, goal->property.header.frame_id);
  }
  catch (const tf::TransformException& ex)
  {
    action_server_.setAborted(Server::Result(),
                              std::string("Boustrophedon planner failed with a tf exception: ") + ex.what());
    return;
  }

  std::vector<NavPoint> path;

  auto preprocess_transform = preprocessPolygon(polygon, robot_position, stripe_angle_);

  Polygon fill_polygon;
  outline_planner_.addToPath(polygon, robot_position, path, fill_polygon);

  PolygonDecomposer polygon_decomposer{};
  polygon_decomposer.decompose(fill_polygon);
  std::vector<Polygon> sub_polygons = polygon_decomposer.getSubPolygons(robot_position);

  for (const auto& subpoly : sub_polygons)
  {
    striping_planner_.addToPath(subpoly, robot_position, path);
  }

  postprocessPolygonAndPath(preprocess_transform, polygon, path);

  auto result = toResult(std::move(path), boundary_frame);
  action_server_.setSucceeded(result);
}

boustrophedon_msgs::PlanMowingPathResult BoustrophedonPlannerServer::toResult(std::vector<NavPoint>&& path,
                                                                              const std::string& frame) const
{
  boustrophedon_msgs::PlanMowingPathResult result;
  result.plan.points.reserve(path.size());
  result.plan.header.stamp = ros::Time::now();
  result.plan.header.frame_id = frame;

  for (const auto& point : path)
  {
    boustrophedon_msgs::StripingPoint stripe_point{};
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    stripe_point.point.z = 0;
    stripe_point.type = static_cast<uint8_t>(point.type);
    result.plan.points.emplace_back(stripe_point);
  }
  return result;
}

Polygon BoustrophedonPlannerServer::fromBoundary(const geometry_msgs::PolygonStamped& boundary) const
{
  Polygon polygon;
  for (const auto& point : boundary.polygon.points)
  {
    polygon.push_back(Point(point.x, point.y));
  }
  return polygon;
}

Point BoustrophedonPlannerServer::fromPositionWithFrame(const geometry_msgs::PoseStamped& pose,
                                                        const std::string& target_frame) const
{
  geometry_msgs::PoseStamped transformed_pose;
  transform_listener_.transformPose(target_frame, pose, transformed_pose);
  return { transformed_pose.pose.position.x, transformed_pose.pose.position.y };
}

bool BoustrophedonPlannerServer::convertStripingPlanToPath(boustrophedon_msgs::ConvertPlanToPath::Request& request,
                                                           boustrophedon_msgs::ConvertPlanToPath::Response& response)
{
  response.path.header.frame_id = request.plan.header.frame_id;
  response.path.header.stamp = request.plan.header.stamp;

  std::transform(request.plan.points.begin(), request.plan.points.end(), response.path.poses.begin(),
                 [&](const boustrophedon_msgs::StripingPoint& point) {
                   geometry_msgs::PoseStamped pose;
                   pose.header.frame_id = request.plan.header.frame_id;
                   pose.header.stamp = request.plan.header.stamp;
                   pose.pose.position = point.point;
                   pose.pose.orientation.x = 0.0;
                   pose.pose.orientation.y = 0.0;
                   pose.pose.orientation.z = 0.0;
                   pose.pose.orientation.w = 1.0;
                   return pose;
                 });
  return true;
}
