#include "boustrophedon_client/boustrophedon_planner_client.h"

#include <thread>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;

BoustrophedonPlannerClient::BoustrophedonPlannerClient() : Node("boustrophedon_planner_client")
{
  this->client_ptr_ = rclcpp_action::create_client<PlanMowingPathAction>(this, "plan_path");

  this->timer_ =
      this->create_wall_timer(chrono::milliseconds(500), bind(&BoustrophedonPlannerClient::send_goal, this));

  declare_parameter("boundary_file", "boundary.txt");
  boundary_file = get_parameter("boundary_file").as_string();
}

void writeToTextFile(string filename, stringstream& stream)
{
  ofstream file;

  file.open(filename.c_str(), ios::trunc);
  if (!file)
  {
    string errorMsg = "Unable to open file " + filename;
    throw runtime_error(errorMsg.c_str());
  }

  file << stream.rdbuf();
  file.close();
}

vector<string> readTextFile(string& filename)
{
  ifstream file;

  file.open(filename.c_str());
  if (!file)
  {
    string errorMsg = "Unable to open file " + filename;
    throw runtime_error(errorMsg.c_str());
  }

  string line;
  vector<string> lines;

  while (getline(file, line))
  {
    lines.push_back(line);
  }

  file.close();
  return lines;
}

void BoustrophedonPlannerClient::send_goal()
{
  using namespace placeholders;

  this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server())
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = PlanMowingPathAction::Goal();

  // Map boundary
  geometry_msgs::msg::PolygonStamped map_stamped;
  geometry_msgs::msg::Polygon map;

  RCLCPP_INFO(get_logger(), "Reading map boundary from %s", boundary_file.c_str());

  vector<string> lines = readTextFile(boundary_file);

  for (vector<string>::iterator line = lines.begin(); line != lines.end(); ++line)
  {
    float x, y;
    int n = sscanf(line->c_str(), "(%f, %f)", &x, &y);
    if (n != 2)
    {
      throw runtime_error("Unable to parse " + boundary_file);
    }

    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = 0;
    map.points.push_back(point);
  }

  RCLCPP_INFO(get_logger(), "Creating polygon (%ld):", map.points.size());

  map_stamped.polygon = map;
  goal_msg.property = map_stamped;

  // Robot initial position
  geometry_msgs::msg::PoseStamped robot_pose_stamped;
  geometry_msgs::msg::Pose robot_pose;
  geometry_msgs::msg::Point robot_position;
  robot_position.x = 0;
  robot_position.y = 0;
  robot_position.z = 0;
  robot_pose.position = robot_position;

  // Robot initial orientation
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion robot_orientation = tf2::toMsg(tf2_quat);
  robot_pose.orientation = robot_orientation;

  robot_pose_stamped.pose = robot_pose;
  goal_msg.robot_position = robot_pose_stamped;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<PlanMowingPathAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = bind(&BoustrophedonPlannerClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = bind(&BoustrophedonPlannerClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = bind(&BoustrophedonPlannerClient::result_callback, this, _1);

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void BoustrophedonPlannerClient::goal_response_callback(const GoalHandlePlanMowingPathAction::SharedPtr & goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void BoustrophedonPlannerClient::feedback_callback(GoalHandlePlanMowingPathAction::SharedPtr,
                                                   const std::shared_ptr<const PlanMowingPathAction::Feedback>)
{
  RCLCPP_INFO(this->get_logger(), "Unexpected feedback received");
}

void BoustrophedonPlannerClient::result_callback(const GoalHandlePlanMowingPathAction::WrappedResult & result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received:");
  stringstream ss;
  for (auto point : result.result->plan.points)
  {
    ss << "(" << fixed << setprecision(2) << point.point.x << ", " << fixed << setprecision(2) << point.point.y << ")\n";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());

  RCLCPP_INFO(this->get_logger(), "Writing result to path.txt");

  writeToTextFile("path.txt", ss);

  rclcpp::shutdown();
}