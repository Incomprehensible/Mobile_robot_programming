#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define SQUARE_POLYGON 5.0
#define DEFAULT_SPEED 1.0

class TurtleBot3Subscriber : public rclcpp::Node
{   
    public:
        explicit TurtleBot3Subscriber(const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "subscriber_node");

    private:
        void pose_callback(const geometry_msgs::msg::PoseStamped&);

        
        double distance;
        // tf2::Quaternion orientation_;
        tf2::Vector3 prev_position_;
        
        // rclcpp::TimerBase::SharedPtr timer_;
        
        // tf2::BufferCore tf_buffer_;
        // tf2_ros::TransformListener tf_listener_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

#endif