#include "rclcpp/rclcpp.hpp"
#include "boustrophedon_client/boustrophedon_planner_client.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoustrophedonPlannerClient>());
  rclcpp::shutdown();
  return 0;
}
