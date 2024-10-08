#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "speed_interface/srv/set_speed.hpp" 

// constants
#define SQUARE_POLYGON 5.0
#define DEFAULT_SPEED 1.0
#define MAX_SPEED 2.0

class TurtleBot3Controller : public rclcpp::Node
{
    struct Setpoint {
        double x;
        double y;
        double yaw;
    };

    // P-controller gains
    // gain for the linear motion control
    const double K_l = 1.0;
    // gain for the heading angle control
    const double K_ha = 10.0;
    // gain for the turning angle control
    const double K_ta = 0.9;

    // position tolerances
    const double distanceTolerance = 0.1;
    const double headingAngleTolerance = 0.05;
    const double turnAngleTolerance = 0.01;
    
    public:
        explicit TurtleBot3Controller(const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "control_node");
        void set_parameters();
        void init_setpoints();

    private:
        void set_speed(const std::shared_ptr<speed_interface::srv::SetSpeed::Request>);

        void go_in_square();

        geometry_msgs::msg::TransformStamped::SharedPtr get_position();
        double normalize_angle(double);

        double get_linear_velocity(double, double);
        double get_angular_velocity(double);
        double get_angular_turn_velocity(double);

        void control_cycle();
        void turn_control_cycle();

        void send_velocity();
        void publish_pose();
        rcl_interfaces::msg::SetParametersResult param_change_callback(const std::vector<rclcpp::Parameter>&);

        // private data
        std::vector<Setpoint> setpoints_;
        Setpoint goal_;
        double speed_;
        bool goal_success_;
        bool angle_goal_success_;
        tf2::Vector3 linear_vel_; 
        tf2::Vector3 angular_vel_;
        tf2::Quaternion orientation_;
        tf2::Vector3 position_;
        // timers
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr vel_timer_;
        rclcpp::TimerBase::SharedPtr pose_timer_;
        // coordinates transform
        tf2::BufferCore tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        // publishers, subscribers, service
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        rclcpp::Service<speed_interface::srv::SetSpeed>::SharedPtr speed_service_;
};

#endif
