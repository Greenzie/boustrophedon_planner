#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "boustrophedon_server/boustrophedon_planner_server.h"

BoustrophedonPlannerServer::BoustrophedonPlannerServer()
  : private_node_handle_("~")
  , action_server_(node_handle_, "plan_path", boost::bind(&BoustrophedonPlannerServer::executePlanPathAction, this, _1),
                   false)
  , conversion_server_{ node_handle_.advertiseService("convert_striping_plan_to_path",
                                                      &BoustrophedonPlannerServer::convertStripingPlanToPath, this) }
{
  std::size_t error = 0;
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "repeat_boundary", repeat_boundary_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_clockwise", outline_clockwise_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "skip_outlines", skip_outlines_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_layer_count", outline_layer_count_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_separation", stripe_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "intermediary_separation", intermediary_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_angle", stripe_angle_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get("plan_path", private_node_handle_,
                                                             "enable_stripe_angle_orientation", enable_orientation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "travel_along_boundary", travel_along_boundary_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get(
      "plan_path", private_node_handle_, "allow_points_outside_boundary", allow_points_outside_boundary_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "enable_half_y_turns", enable_half_y_turns_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "points_per_turn", points_per_turn_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "turn_start_offset", turn_start_offset_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "publish_polygons", publish_polygons_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "publish_path_points", publish_path_points_));
  rosparam_shortcuts::shutdownIfError("plan_path", error);

  if (intermediary_separation_ <= 0.0)
  {
    // doesn't make sense, or we don't want intermediaries. set it to double max so we can't make any intermediaries
    intermediary_separation_ = std::numeric_limits<double>::max();
  }

  if (enable_half_y_turns_ && outline_layer_count_ < 1)
  {
    if (allow_points_outside_boundary_)
    {
      ROS_WARN_STREAM("Current configuration will result in turns that go outside the boundary, but this has been "
                      "explicitly enabled");
    }
    else
    {
      // we can't do half-y-turns safely without an inner boundary layer, as the arc will stick outside of the boundary
      ROS_ERROR_STREAM("Cannot plan using half-y-turns if the outline_layer_count is less than 1! Boustrophedon "
                       "planner will not start.");
      return;
    }
  }

  striping_planner_.setParameters({ stripe_separation_, intermediary_separation_, travel_along_boundary_,
                                    enable_half_y_turns_, points_per_turn_, turn_start_offset_ });
  outline_planner_.setParameters(
      { repeat_boundary_, outline_clockwise_, skip_outlines_, outline_layer_count_, stripe_separation_ });

  action_server_.start();

  if (publish_polygons_)
  {
    initial_polygon_publisher_ = private_node_handle_.advertise<geometry_msgs::PolygonStamped>("initial_polygon", 1);
    preprocessed_polygon_publisher_ =
        private_node_handle_.advertise<geometry_msgs::PolygonStamped>("preprocessed_polygon", 1);
  }
  // mainly for use with plotJuggler, which wants the points to be put one at a time on the same topic
  if (publish_path_points_)
  {
    path_points_publisher_ = private_node_handle_.advertise<geometry_msgs::PointStamped>("path_points", 1000);
    polygon_points_publisher_ = private_node_handle_.advertise<geometry_msgs::PointStamped>("polygon_points", 1000);
  }
}

void BoustrophedonPlannerServer::executePlanPathAction(const boustrophedon_msgs::PlanMowingPathGoalConstPtr& goal)
{
  std::string boundary_frame = goal->property.header.frame_id;
  if (enable_orientation_)
  {
    stripe_angle_ = getStripeAngleFromOrientation(goal->robot_position);
  }

  Polygon polygon = fromBoundary(goal->property);
  if (!checkPolygonIsValid(polygon))
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner does not work for polygons of "
                                                            "size "
                                                            "< 3."));
    return;
  }
  if (!polygon.is_simple())
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner only works for simple (non "
                                                            "self-intersecting) polygons."));
    return;
  }
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

  if (publish_polygons_)
  {
    initial_polygon_publisher_.publish(goal->property);
    preprocessed_polygon_publisher_.publish(convertCGALPolygonToMsg(polygon));
  }

  Polygon fill_polygon;
  if (!outline_planner_.addToPath(polygon, robot_position, path, fill_polygon))
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner failed because "
                                                            "outline_layer_count "
                                                            "was too large for the boundary."));
    return;
  }
  PolygonDecomposer polygon_decomposer{};
  // A print statement MUST be here, see issue #1586
  ROS_INFO_STREAM("Decomposing boundary polygon into sub-polygons...");
  polygon_decomposer.decompose(fill_polygon);
  std::vector<Polygon> sub_polygons = polygon_decomposer.getSubPolygons(robot_position);
  ROS_INFO_STREAM("Broke the boundary up into " << sub_polygons.size() << " sub-polygons");
  Polygon merged_polygon;
  Point start_position = robot_position;
  for (const auto& subpoly : sub_polygons)
  {
    // combine the new subpoly with the merged_polygon. If merged_polygon is empty, it returns the sub polygon
    merged_polygon = mergePolygons(merged_polygon, subpoly);

    // add the stripes to the path, using merged_polygon boundary to travel if necessary.
    striping_planner_.addToPath(merged_polygon, subpoly, robot_position, path);
  }
  if (travel_along_boundary_)
  {
    striping_planner_.addReturnToStart(merged_polygon, start_position, robot_position, path);
  }
  postprocessPolygonAndPath(preprocess_transform, polygon, path);
  if (publish_path_points_)  // if we care about visualizing the planned path in plotJuggler
  {
    publishPathPoints(path);
    publishPolygonPoints(polygon);
  }
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

geometry_msgs::PolygonStamped BoustrophedonPlannerServer::convertCGALPolygonToMsg(const Polygon& poly) const
{
  geometry_msgs::PolygonStamped stamped_poly;

  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::Point32 point;
    point.x = float(it->x());
    point.y = float(it->y());
    point.z = float(0.0);
    stamped_poly.polygon.points.push_back(point);
  }

  stamped_poly.header.frame_id = "map";
  stamped_poly.header.stamp = ros::Time::now();
  return stamped_poly;
}

bool BoustrophedonPlannerServer::checkPolygonIsValid(const Polygon& poly) const
{
  return !(poly.size() < 3);  // expand later if we find more cases of invalid polygons
}

// get the yaw from the robot_position part of the given goal
double BoustrophedonPlannerServer::getStripeAngleFromOrientation(const geometry_msgs::PoseStamped& robot_position)
{
  tf::Quaternion quat(robot_position.pose.orientation.x, robot_position.pose.orientation.y,
                      robot_position.pose.orientation.z, robot_position.pose.orientation.w);
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO_STREAM("Got Striping Angle from Orientation: " << yaw);
  // TODO: Recalibrate the IMU so that we can get rid of this constant below.
  return yaw + 1.57079632679;  // Adds PI / 2 to account for incorrect IMU calibration / reference vector
}

// publishes the path points one at a time for visualization in plotJuggler
void BoustrophedonPlannerServer::publishPathPoints(const std::vector<NavPoint>& path) const
{
  for (NavPoint point : path)
  {
    geometry_msgs::PointStamped stripe_point{};
    stripe_point.header.stamp = ros::Time::now();
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    path_points_publisher_.publish(stripe_point);
    ros::spinOnce();
  }
}

// publishes the polygon points one at a time for visualization in plotJuggler
void BoustrophedonPlannerServer::publishPolygonPoints(const Polygon& poly) const
{
  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::PointStamped point;
    point.header.stamp = ros::Time::now();
    point.point.x = float(it->x());
    point.point.y = float(it->y());
    point.point.z = float(0.0);
    polygon_points_publisher_.publish(point);
    ros::spinOnce();
  }
}