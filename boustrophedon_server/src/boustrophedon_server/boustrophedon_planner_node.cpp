#include <rclcpp/rclcpp.hpp>

#include "boustrophedon_server/boustrophedon_planner_server.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoustrophedonPlannerServer>());
  rclcpp::shutdown();
  return 0;
}
