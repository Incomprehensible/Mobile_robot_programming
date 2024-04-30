#include "subscriber_node.hpp"
// #include <angles/angles.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

TurtleBot3Subscriber::TurtleBot3Subscriber(const rclcpp::NodeOptions & options, const std::string & node_name)
    : Node(node_name, options), distance(0.0)
{
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.durability_volatile();
    // 6D position subscriber
    this->pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", qos, std::bind(&TurtleBot3Subscriber::pose_callback, this, std::placeholders::_1));
}

void TurtleBot3Subscriber::pose_callback(const geometry_msgs::msg::PoseStamped& pose)
{
    double roll, pitch, yaw;

    // RCLCPP_INFO(this->get_logger(), "DBG: Current running distance: %f", distance);
    // RCLCPP_INFO(this->get_logger(), "DBG: Current pose: %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    // RCLCPP_INFO(this->get_logger(), "DBG: Old pose: %f %f %f", prev_position_.getX(), prev_position_.getY(), prev_position_.getZ());

    tf2::Quaternion curr_orientation;
    tf2::fromMsg(pose.pose.orientation, curr_orientation);
    tf2::getEulerYPR(curr_orientation, yaw, pitch, roll);
    RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

    tf2::Vector3 curr_pose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    // RCLCPP_INFO(this->get_logger(), "DBG: dDistance: %f", tf2::tf2Distance(this->prev_position_, curr_pose));
    this->distance += tf2::tf2Distance(this->prev_position_, curr_pose);
    RCLCPP_INFO(this->get_logger(), "Total running distance: %f", distance);


    this->prev_position_ = curr_pose;
    // this->prev_position_.setX(pose.pose.position.x);
    // this->prev_position_.setY(pose.pose.position.y);
    // this->prev_position_.setZ(pose.pose.position.z);
}
