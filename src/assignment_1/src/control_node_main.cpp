#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "control_node.hpp"

#define LOOP_RATE 0.5

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 3.0);
  geometry_msgs::msg::Quaternion msg = tf2::toMsg(orientation);
  double roll, pitch, yaw;
  tf2::getEulerYPR(msg, yaw, pitch, roll);

  RCLCPP_INFO(rclcpp::get_logger("NAD"), "Roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

  rclcpp::spin(std::make_shared<TurtleBot3Controller>());

  rclcpp::shutdown();

  return 0;
}