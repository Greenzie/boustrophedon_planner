#include "boustrophedon_server/boustrophedon_planner_server.h"

#include <thread>

BoustrophedonPlannerServer::BoustrophedonPlannerServer() : Node("boustrophedon_planner_server")
{
  using namespace std::placeholders;

  RCLCPP_INFO(get_logger(), "Initializing node boustrophedon_server");

  RCLCPP_INFO(get_logger(), "Creating action server");
  action_server_ = rclcpp_action::create_server<PlanMowingPathAction>(
      this, "plan_path", std::bind(&BoustrophedonPlannerServer::handleGoal, this, _1, _2),
      std::bind(&BoustrophedonPlannerServer::handleCancel, this, _1),
      std::bind(&BoustrophedonPlannerServer::handleAccepted, this, _1));

  RCLCPP_INFO(get_logger(), "Creating conversion server");
  conversion_server_ = create_service<ConvertPlanToPathSrv>(
      "convert_striping_plan_to_path", std::bind(&BoustrophedonPlannerServer::convertStripingPlanToPath, this, _1, _2));

  RCLCPP_INFO(get_logger(), "Declaring parameters");
  declare_parameter("repeat_boundary", false);
  repeat_boundary_ = get_parameter("repeat_boundary").as_bool();

  declare_parameter("outline_clockwise", true);
  outline_clockwise_ = get_parameter("outline_clockwise").as_bool();

  declare_parameter("skip_outlines", true);
  skip_outlines_ = get_parameter("skip_outlines").as_bool();

  declare_parameter("outline_layer_count", 0);
  outline_layer_count_ = get_parameter("outline_layer_count").as_int();

  declare_parameter("stripe_separation", 1.0);
  stripe_separation_ = get_parameter("stripe_separation").as_double();

  declare_parameter("intermediary_separation", 0.0);
  intermediary_separation_ = get_parameter("intermediary_separation").as_double();

  declare_parameter("stripe_angle", 0.0);
  stripe_angle_ = get_parameter("stripe_angle").as_double();

  declare_parameter("stripe_angle_from_robot_orientation", false);
  stripe_angle_from_robot_orientation = get_parameter("stripe_angle_from_robot_orientation").as_bool();

  declare_parameter("stripe_angle_from_boundary_orientation", false);
  stripe_angle_from_boundary_orientation = get_parameter("stripe_angle_from_boundary_orientation").as_bool();

  declare_parameter("travel_along_boundary", true);
  travel_along_boundary_ = get_parameter("travel_along_boundary").as_bool();

  declare_parameter("allow_points_outside_boundary", false);
  allow_points_outside_boundary_ = get_parameter("allow_points_outside_boundary").as_bool();

  declare_parameter("enable_half_y_turns", false);
  enable_half_y_turns_ = get_parameter("enable_half_y_turns").as_bool();

  declare_parameter("points_per_turn", 15);
  points_per_turn_ = get_parameter("points_per_turn").as_int();

  declare_parameter("turn_start_offset", 0.5);
  turn_start_offset_ = get_parameter("turn_start_offset").as_double();

  declare_parameter("publish_polygons", true);
  publish_polygons_ = get_parameter("publish_polygons").as_bool();

  declare_parameter("publish_path_points", true);
  publish_path_points_ = get_parameter("publish_path_points").as_bool();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "Verifying parameters");

  if (intermediary_separation_ <= 0.0)
  {
    // doesn't make sense, or we don't want intermediaries. set it to double max so we can't make any intermediaries
    intermediary_separation_ = std::numeric_limits<double>::max();
  }

  if (enable_half_y_turns_ && outline_layer_count_ < 1)
  {
    if (allow_points_outside_boundary_)
    {
      RCLCPP_WARN(get_logger(), "Current configuration will result in turns that go outside the boundary, but this has "
                                "been explicitly enabled");
    }
    else
    {
      // we can't do half-y-turns safely without an inner boundary layer, as the arc will stick outside of the boundary
      RCLCPP_ERROR(get_logger(), "Cannot plan using half-y-turns if the outline_layer_count is less than 1! "
                                 "Boustrophedon planner will not start.");
      return;
    }
  }

  striping_planner_.setParameters({ stripe_separation_, intermediary_separation_, travel_along_boundary_,
                                    enable_half_y_turns_, points_per_turn_, turn_start_offset_ });
  outline_planner_.setParameters(
      { repeat_boundary_, outline_clockwise_, skip_outlines_, outline_layer_count_, stripe_separation_ });

  if (publish_polygons_)
  {
    RCLCPP_INFO(get_logger(), "Creating polygon publisher");
    initial_polygon_publisher_ = create_publisher<geometry_msgs::msg::PolygonStamped>("initial_polygon", 1);
    preprocessed_polygon_publisher_ = create_publisher<geometry_msgs::msg::PolygonStamped>("preprocessed_polygon", 1);
  }
  // mainly for use with plotJuggler, which wants the points to be put one at a time on the same topic
  if (publish_path_points_)
  {
    RCLCPP_INFO(get_logger(), "Creating polygon publisher");
    path_points_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("path_points", 1000);
    polygon_points_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("polygon_points", 1000);
  }

  RCLCPP_INFO(get_logger(), "Node boustrophedon_server successfully initialized");
}

rclcpp_action::GoalResponse BoustrophedonPlannerServer::handleGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PlanMowingPathAction::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
BoustrophedonPlannerServer::handleCancel(const std::shared_ptr<GoalHandlePlanMowingPathAction> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BoustrophedonPlannerServer::handleAccepted(const std::shared_ptr<GoalHandlePlanMowingPathAction> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&BoustrophedonPlannerServer::executePlanPathAction, this, _1), goal_handle }.detach();
}

void BoustrophedonPlannerServer::executePlanPathAction(
    const std::shared_ptr<GoalHandlePlanMowingPathAction> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();

  auto result = std::make_shared<PlanMowingPathAction::Result>();

  std::string boundary_frame = goal->property.header.frame_id;

  Polygon polygon = fromBoundary(goal->property);

  RCLCPP_INFO(get_logger(), "Boundary limits (%ld):", goal->property.polygon.points.size());
  for (Polygon::iterator vertex = polygon.begin(); vertex != polygon.end(); ++vertex)
  {
    RCLCPP_INFO(get_logger(), "(%.2f, %.2f)", vertex->x(), vertex->y());
  }

  if (!checkPolygonIsValid(polygon))
  {
    RCLCPP_ERROR(get_logger(), "Boustrophedon planner does not work for polygons of size < 3.");
    goal_handle->abort(result);
    return;
  }
  if (!polygon.is_simple())
  {
    RCLCPP_ERROR(get_logger(), "Boustrophedon planner only works for simple (non self-intersecting) polygons.");
    goal_handle->abort(result);
    return;
  }

  if (stripe_angle_from_robot_orientation && !stripe_angle_from_boundary_orientation)
  {
    stripe_angle_ = getStripeAngleFromOrientation(goal->robot_position);
  }
  else if (!stripe_angle_from_robot_orientation && stripe_angle_from_boundary_orientation)
  {
    double orientation_wrt_x_axis = getPolygonOrientation(polygon);
    RCLCPP_INFO(get_logger(), "Boundary orientation: %.2f", orientation_wrt_x_axis);
    stripe_angle_ = M_PI / 2.0 - orientation_wrt_x_axis;
  }
  else if (stripe_angle_from_robot_orientation && stripe_angle_from_boundary_orientation)
  {
    RCLCPP_ERROR(get_logger(), "Both stripe_angle_from_robot_orientation and stripe_angle_from_boundary_orientation "
                               "should not be enabled at the same time.");
    goal_handle->abort(result);
    return;
  }

  Point robot_position;
  try
  {
    robot_position = fromPositionWithFrame(goal->robot_position, goal->property.header.frame_id);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Boustrophedon planner failed with a tf exception: %s", ex.what());
    goal_handle->abort(result);
    return;
  }
  std::vector<NavPoint> path;

  auto preprocess_transform = preprocessPolygon(polygon, robot_position, stripe_angle_);

  if (publish_polygons_)
  {
    initial_polygon_publisher_->publish(goal->property);
    preprocessed_polygon_publisher_->publish(convertCGALPolygonToMsg(polygon));
  }

  Polygon fill_polygon;
  if (!outline_planner_.addToPath(polygon, robot_position, path, fill_polygon))
  {
    RCLCPP_ERROR(get_logger(), "Boustrophedon planner failed because outline_layer_count was too large for the "
                               "boundary.");
    return;
  }
  PolygonDecomposer polygon_decomposer{};
  // A print statement MUST be here, see issue #1586
  RCLCPP_INFO(get_logger(), "Decomposing boundary polygon into sub-polygons...");
  polygon_decomposer.decompose(fill_polygon);
  std::vector<Polygon> sub_polygons = polygon_decomposer.getSubPolygons(robot_position);
  RCLCPP_INFO(get_logger(), "Broke the boundary up into %ld sub-polygons", sub_polygons.size());
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
  RCLCPP_INFO(get_logger(), "Post processing polygon and path");
  postprocessPolygonAndPath(preprocess_transform, polygon, path);
  if (publish_path_points_)  // if we care about visualizing the planned path in plotJuggler
  {
    RCLCPP_INFO(get_logger(), "Publishing path and polygon");
    publishPathPoints(path);
    publishPolygonPoints(polygon);
  }

  RCLCPP_INFO(get_logger(), "Converting boundary to result");
  *result = toResult(std::move(path), boundary_frame);
  goal_handle->succeed(result);
}

BoustrophedonPlannerServer::PlanMowingPathAction::Result
BoustrophedonPlannerServer::toResult(std::vector<NavPoint>&& path, const std::string& frame) const
{
  PlanMowingPathAction::Result result;
  result.plan.points.reserve(path.size());
  result.plan.header.stamp = now();
  result.plan.header.frame_id = frame;

  for (const auto& point : path)
  {
    StripingPointMsg stripe_point{};
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    stripe_point.point.z = 0;
    stripe_point.type = static_cast<uint8_t>(point.type);
    result.plan.points.emplace_back(stripe_point);
  }
  return result;
}

Polygon BoustrophedonPlannerServer::fromBoundary(const geometry_msgs::msg::PolygonStamped& boundary) const
{
  Polygon polygon;
  for (const auto& point : boundary.polygon.points)
  {
    polygon.push_back(Point(point.x, point.y));
  }
  return polygon;
}

Point BoustrophedonPlannerServer::fromPositionWithFrame(const geometry_msgs::msg::PoseStamped& pose,
                                                        const std::string& target_frame) const
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  tf_buffer_->transform(pose, transformed_pose, target_frame);
  return { transformed_pose.pose.position.x, transformed_pose.pose.position.y };
}

void BoustrophedonPlannerServer::convertStripingPlanToPath(ConvertPlanToPathSrv::Request::SharedPtr request,
                                                           ConvertPlanToPathSrv::Response::SharedPtr response)
{
  response->path.header.frame_id = request->plan.header.frame_id;
  response->path.header.stamp = request->plan.header.stamp;

  std::transform(request->plan.points.begin(), request->plan.points.end(), response->path.poses.begin(),
                 [&](const StripingPointMsg& point) {
                   geometry_msgs::msg::PoseStamped pose;
                   pose.header.frame_id = request->plan.header.frame_id;
                   pose.header.stamp = request->plan.header.stamp;
                   pose.pose.position = point.point;
                   pose.pose.orientation.x = 0.0;
                   pose.pose.orientation.y = 0.0;
                   pose.pose.orientation.z = 0.0;
                   pose.pose.orientation.w = 1.0;
                   return pose;
                 });
}

geometry_msgs::msg::PolygonStamped BoustrophedonPlannerServer::convertCGALPolygonToMsg(const Polygon& poly) const
{
  geometry_msgs::msg::PolygonStamped stamped_poly;

  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::msg::Point32 point;
    point.x = float(it->x());
    point.y = float(it->y());
    point.z = float(0.0);
    stamped_poly.polygon.points.push_back(point);
  }

  stamped_poly.header.frame_id = "map";
  stamped_poly.header.stamp = now();
  return stamped_poly;
}

bool BoustrophedonPlannerServer::checkPolygonIsValid(const Polygon& poly) const
{
  return !(poly.size() < 3);  // expand later if we find more cases of invalid polygons
}

// get the yaw from the robot_position part of the given goal
double BoustrophedonPlannerServer::getStripeAngleFromOrientation(const geometry_msgs::msg::PoseStamped& robot_position)
{
  tf2::Quaternion quat(robot_position.pose.orientation.x, robot_position.pose.orientation.y,
                       robot_position.pose.orientation.z, robot_position.pose.orientation.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  RCLCPP_INFO(get_logger(), "Got Striping Angle from Orientation: %.2f", yaw);
  // TODO: Recalibrate the IMU so that we can get rid of this constant below.
  return yaw + 1.57079632679;  // Adds PI / 2 to account for incorrect IMU calibration / reference vector
}

// get the average polygon orientation
double BoustrophedonPlannerServer::getPolygonOrientation(const Polygon& polygon)
{
  double best_angle = 0;
  double min_intersections = DBL_MAX;

  for (double angle_candidate = 0; angle_candidate < M_PI; angle_candidate += M_PI / 180)
  {
    Direction dir(1, tan(angle_candidate));
    int n_intersections = findNIntersections(polygon, dir, stripe_separation_);

    if (n_intersections < min_intersections)
    {
      best_angle = angle_candidate;
      min_intersections = n_intersections;
    }
  }

  return best_angle;
}

// publishes the path points one at a time for visualization in plotJuggler
void BoustrophedonPlannerServer::publishPathPoints(const std::vector<NavPoint>& path)
{
  for (NavPoint point : path)
  {
    geometry_msgs::msg::PointStamped stripe_point{};
    stripe_point.header.stamp = now();
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    path_points_publisher_->publish(stripe_point);
  }
}

// publishes the polygon points one at a time for visualization in plotJuggler
void BoustrophedonPlannerServer::publishPolygonPoints(const Polygon& poly)
{
  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::msg::PointStamped point;
    point.header.stamp = now();
    point.point.x = float(it->x());
    point.point.y = float(it->y());
    point.point.z = float(0.0);
    polygon_points_publisher_->publish(point);
  }
}