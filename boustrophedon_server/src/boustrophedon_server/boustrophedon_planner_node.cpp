#include <ros/ros.h>
#include "boustrophedon_server/boustrophedon_planner_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boustrophedon_planner_node");

  BoustrophedonPlannerServer server;

  ros::spin();

  return 0;
}
