#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z);

// server has a service to convert StripingPlan to Path, but all it does it call this method
bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path)
{
  path.header.frame_id = striping_plan.header.frame_id;
  path.header.stamp = striping_plan.header.stamp;

  path.poses.clear();

  // path.poses.resize(striping_plan.points.size());
  // std::transform(striping_plan.points.begin(), striping_plan.points.end(), path.poses.begin(),
  //                [&](const boustrophedon_msgs::StripingPoint& point) {
  //                  geometry_msgs::PoseStamped pose;
  //                  pose.header.frame_id = striping_plan.header.frame_id;
  //                  pose.header.stamp = striping_plan.header.stamp;
  //                  pose.pose.position = point.point;
  //                  pose.pose.orientation.x = 0.0;
  //                  pose.pose.orientation.y = 0.0;
  //                  pose.pose.orientation.z = 0.0;
  //                  pose.pose.orientation.w = 1.0;
  //                  return pose;
  //                });

  for (std::size_t i = 0; i < striping_plan.points.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = striping_plan.header.frame_id;
    pose.header.stamp = striping_plan.header.stamp;
    pose.pose.position = striping_plan.points[i].point;

    if (i < striping_plan.points.size() - 1)
    {
      double dx = striping_plan.points[i + 1].point.x - striping_plan.points[i].point.x;
      double dy = striping_plan.points[i + 1].point.y - striping_plan.points[i].point.y;
      double dz = striping_plan.points[i + 1].point.z - striping_plan.points[i].point.z;

      pose.pose.orientation = headingToQuaternion(dx, dy, dz);
    }
    else
    {
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
    }

    path.poses.push_back(pose);
  }

  return true;
}

bool got_initial_pose = false;
geometry_msgs::PoseStamped initial_pose;
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr init_pose)
{
  initial_pose.header = init_pose->header;
  initial_pose.pose = init_pose->pose.pose;
  got_initial_pose = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boustrophedon_planner_client");
  ros::NodeHandle n;

  actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client("plan_path",
                                                                                 true);  // server name and spin thread

  ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("/input_polygon", 1, true);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/result_path", 1, true);
  ros::Publisher start_pub = n.advertise<geometry_msgs::PoseStamped>("/start_pose", 1, true);
  ros::Subscriber init_pose =
      n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseCallback);

  ros::Rate loop_rate(10);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  client.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started");

  boustrophedon_msgs::PlanMowingPathGoal goal;

  goal.property.header.stamp = ros::Time::now();
  goal.property.header.frame_id = "map";

  goal.property.polygon.points.resize(4);
  goal.property.polygon.points[0].x = 0;
  goal.property.polygon.points[0].y = 0;
  goal.property.polygon.points[1].x = 0;
  goal.property.polygon.points[1].y = 10;
  goal.property.polygon.points[2].x = 10;
  goal.property.polygon.points[2].y = 10;
  goal.property.polygon.points[3].x = 10;
  goal.property.polygon.points[3].y = 0;

  goal.robot_position.pose.orientation.w = 1.0;

  polygon_pub.publish(goal.property);

  ROS_INFO_STREAM("Waiting for goal");

  while (ros::ok())
  {
    if (got_initial_pose)
    {
      goal.robot_position = initial_pose;
      // goal.robot_position.header = goal.property.header;
      // goal.robot_position.pose.position.x = 1.0;
      // goal.robot_position.pose.position.y = 1.0;

      start_pub.publish(goal.robot_position);

      client.sendGoal(goal);
      ROS_INFO_STREAM("Sending goal");

      // wait for the action to return
      bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

      if (!finished_before_timeout)
      {
        ROS_INFO("Action did not finish before the time out.");
        continue;
      }

      actionlib::SimpleClientGoalState state = client.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
      boustrophedon_msgs::PlanMowingPathResultConstPtr result = client.getResult();
      std::cout << "Result with : " << result->plan.points.size() << std::endl;

      nav_msgs::Path path;
      convertStripingPlanToPath(result->plan, path);

      path_pub.publish(path);

      got_initial_pose = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z)
{
  // get orientation from heading vector
  const tf2::Vector3 heading_vector(x, y, z);
  const tf2::Vector3 origin(1, 0, 0);

  const auto w = (origin.length() * heading_vector.length()) + tf2::tf2Dot(origin, heading_vector);
  const tf2::Vector3 a = tf2::tf2Cross(origin, heading_vector);
  tf2::Quaternion q(a.x(), a.y(), a.z(), w);
  q.normalize();

  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w()))
  {
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    q.setW(1);
  }

  return tf2::toMsg(q);
}
