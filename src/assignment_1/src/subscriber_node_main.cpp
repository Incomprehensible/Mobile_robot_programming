#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "subscriber_node.hpp"

#define LOOP_RATE 0.5

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TurtleBot3Subscriber>());

  rclcpp::shutdown();

  return 0;
}