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
// #include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define SQUARE_POLYGON 5

class TurtleBot3Controller : public rclcpp::Node
{
    // struct Pose6D {
    //     double x;
    //     double y;
    //     double z;
    //     double roll;
    //     double pitch;
    //     double yaw;
    // };
    enum State {
        REST,
        FORWARD,
        TURN,
        STOP,
    };
    // The gain K, which is used to calculate the linear velocity
    const double K_l = 1.0;
 
    // The distance threshold in meters that will determine when 
    // the turtlesim robot successfully reaches the goal.
    const double distanceTolerance = 0.01;
    const double angleTolerance = 0.01;
    
    public:
        explicit TurtleBot3Controller(const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "control_node");
        void set_parameters();

        // test
        void test_get_pose();

    private:
        geometry_msgs::msg::TransformStamped::SharedPtr get_position();
        double get_euclidian_distance(double, double);
        double get_linear_distance(double);
        double get_angular_distance(double);
        double set_linear_velocity(double);
        double set_linear_velocity2(double, double);
        double set_angular_velocity(double);
        void control_cycle();
        // void go_forward();
        // void turn();
        // void stop();
        void send_velocity();
        void publish_pose();
        rcl_interfaces::msg::SetParametersResult param_change_callback(const std::vector<rclcpp::Parameter>&);
        // void reset_timers();
        // void activate_timers();

        // test
        // rclcpp::TimerBase::SharedPtr go_timer_;
        // rclcpp::TimerBase::SharedPtr turn_timer_;

        State state;
        double x_init_;
        double y_init_;
        double yaw_init_;
        double speed_;
        // Pose6D pose_;
        tf2::Vector3 linear_vel_; 
        tf2::Vector3 angular_vel_;
        tf2::Quaternion orientation_;
        tf2::Vector3 position_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr vel_timer_;
        rclcpp::TimerBase::SharedPtr pose_timer_;
        tf2::BufferCore tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

};

#endif
