#include "control_node_mb.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>
#include <angles/angles.h>

using namespace std::chrono_literals;

TurtleBot3Controller::TurtleBot3Controller(const rclcpp::NodeOptions & options, const std::string & node_name)
    : Node(node_name, options), tf_buffer_(), tf_listener_(tf_buffer_)
{
    rcl_interfaces::msg::ParameterDescriptor speed_descriptor;
    speed_descriptor.description = "Constant speed for the TurtleBot3";

    this->declare_parameter<double>("speed", 0.0, speed_descriptor);
    this->set_parameters();

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TurtleBot3Controller::param_change_callback, this, std::placeholders::_1));

    linear_vel_ = {0, 0, 0};
    angular_vel_ = {0, 0, 0};

    this->init_setpoints();

    goal_ = setpoints_.front();
    goal_success_ = false;

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(15), std::bind(&TurtleBot3Controller::go_in_square, this));

    // 6D position publisher
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);

    // Control velocity publisher
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    this->vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TurtleBot3Controller::send_velocity, this));
    this->pose_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TurtleBot3Controller::publish_pose, this));
}

void TurtleBot3Controller::set_parameters()
{
    this->speed_ = this->get_parameter("speed").as_double();
}

double TurtleBot3Controller::normalize_angle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

void TurtleBot3Controller::init_setpoints()
{
    rclcpp::Rate rate(5s);

    geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr;
    while ((odom2robot_ptr = this->get_position()) == nullptr) {
        rate.sleep();
    }

    auto odom2robot = *(odom2robot_ptr.get());

    double x_origin = odom2robot.transform.translation.x;
    double y_origin = odom2robot.transform.translation.y;

    orientation_.setX(odom2robot.transform.rotation.x);
    orientation_.setY(odom2robot.transform.rotation.y);
    orientation_.setZ(odom2robot.transform.rotation.z);
    orientation_.setW(odom2robot.transform.rotation.w);
    
    double yaw_origin = tf2::getYaw(orientation_);

    setpoints_.push_back({x_origin+5, y_origin});//, yaw_origin});
    setpoints_.push_back({x_origin+5, y_origin+5});//, normalize_angle(yaw_origin+M_PI_2)});
    setpoints_.push_back({x_origin, y_origin+5});//, normalize_angle(yaw_origin+2*M_PI_2)});
    setpoints_.push_back({x_origin, y_origin});//, normalize_angle(yaw_origin+3*M_PI_2)});
}

geometry_msgs::msg::TransformStamped::SharedPtr TurtleBot3Controller::get_position()
{
    geometry_msgs::msg::TransformStamped odom2robot;

    try {
        odom2robot = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException& e) {
        RCLCPP_ERROR(this->get_logger(), "Odom to robot transform not found: %s", e.what());
        return nullptr;
    }
    return std::make_shared<geometry_msgs::msg::TransformStamped>(odom2robot);
}

double TurtleBot3Controller::get_linear_velocity(double x, double y) {
    double vel_x = 0;
    double dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    RCLCPP_INFO(this->get_logger(), "linear dist: %f", dist);

    if (abs(dist) > distanceTolerance) {
        vel_x = K_l * dist;
        if (abs(vel_x) > this->speed_)
            vel_x = (dist > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_x;
}

// double TurtleBot3Controller::get_angular_distance(double yaw)
// {
//     double shortest_dist = angles::shortest_angular_distance(yaw, goal_yaw_);
    
//     RCLCPP_INFO(this->get_logger(), "yaw: %f, goal_yaw: %f, shortest dist: %f", yaw, goal_yaw_, shortest_dist);

//     return shortest_dist;
// }

double TurtleBot3Controller::get_angular_velocity(double yaw) {
    double vel_theta = 0;
    double theta = atan2(sin(yaw), cos(yaw));

    RCLCPP_INFO(this->get_logger(), "angular dist: %f", theta);

    if (abs(theta) > angleTolerance) {
        vel_theta = K_a * theta;
        if (abs(vel_theta) > this->speed_)
            vel_theta = (theta > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_theta;
}

void TurtleBot3Controller::control_cycle()
{
    if (this->speed_ == 0)
        return;
    auto odom2robot_ptr = this->get_position();
    if (odom2robot_ptr == nullptr)
        return;
    auto odom2robot = *(odom2robot_ptr.get());

    double x = odom2robot.transform.translation.x;
    double y = odom2robot.transform.translation.y;

    orientation_.setX(odom2robot.transform.rotation.x);
    orientation_.setY(odom2robot.transform.rotation.y);
    orientation_.setZ(odom2robot.transform.rotation.z);
    orientation_.setW(odom2robot.transform.rotation.w);
    
    double yaw;
    yaw = tf2::getYaw(orientation_);
    // double roll, pitch, yaw;
    // tf2::getEulerYPR(orientation_, yaw, pitch, roll);

    // RCLCPP_INFO(this->get_logger(), "{x y z yaw}: {%f %f %f %f}", x, y, z, yaw);
    RCLCPP_INFO(this->get_logger(), "x,y: {%f %f}", x, y);
    RCLCPP_INFO(this->get_logger(), "Desired position: {%f %f}", goal_.x, goal_.y);

    double x_diff = goal_.x - x;
    double y_diff = goal_.y - y;
    double yaw_desired = atan2(y_diff, x_diff);
    // double yaw_diff = yaw_desired - yaw;
    double yaw_diff = angles::shortest_angular_distance(yaw, yaw_desired);

    double vel_x = get_linear_velocity(x_diff, y_diff);
    linear_vel_.setX(vel_x);
    if (vel_x == 0)
    {
        angular_vel_.setZ(0);
        goal_success_ = true;
    }
    else
    {
        double vel_theta = get_angular_velocity(yaw_diff);
        angular_vel_.setZ(vel_theta);
    }
}

// void TurtleBot3Controller::control_cycle()
// {
//     if (this->speed_ == 0)
//         return;
//     auto odom2robot_ptr = this->get_position();
//     if (odom2robot_ptr == nullptr)
//         return;
//     auto odom2robot = *(odom2robot_ptr.get());

//     double x = odom2robot.transform.translation.x;
//     double y = odom2robot.transform.translation.y;

//     orientation_.setX(odom2robot.transform.rotation.x);
//     orientation_.setY(odom2robot.transform.rotation.y);
//     orientation_.setZ(odom2robot.transform.rotation.z);
//     orientation_.setW(odom2robot.transform.rotation.w);
    
//     double yaw;
//     yaw = tf2::getYaw(orientation_);
//     // double roll, pitch, yaw;
//     // tf2::getEulerYPR(orientation_, yaw, pitch, roll);

//     // RCLCPP_INFO(this->get_logger(), "{x y z yaw}: {%f %f %f %f}", x, y, z, yaw);
//     RCLCPP_INFO(this->get_logger(), "x,y: {%f %f}", x, y);
//     RCLCPP_INFO(this->get_logger(), "Desired position: {%f %f}", goal_.x, goal_.y);

//     double x_diff = goal_.x - x;
//     double y_diff = goal_.y - y;
//     double yaw_desired = atan2(y_diff, x_diff);
//     // double yaw_diff = yaw_desired - yaw;
//     double yaw_diff = angles::shortest_angular_distance(yaw, yaw_desired);

//     double vel_x = get_linear_velocity(x_diff, y_diff);
//     double vel_theta = get_angular_velocity(yaw_diff);

//     linear_vel_.setX(vel_x);
//     angular_vel_.setZ(vel_theta);

//     if (vel_x == 0 && vel_theta == 0)
//         goal_success_ = true;
// }

void TurtleBot3Controller::go_in_square()
{
    static auto iter = setpoints_.begin();

    if (goal_success_)
    {
        if (iter == setpoints_.end())
            iter = setpoints_.begin();
        else
            iter++;
        goal_ = *iter;
        goal_success_ = false;
        RCLCPP_INFO(this->get_logger(), "GOAL SUCCESS! Next goal: {%f %f}", goal_.x, goal_.y);
    }
    else
        control_cycle();
}

void TurtleBot3Controller::send_velocity()
{
    geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear = tf2::toMsg(linear_vel_);
    cmd_vel.angular = tf2::toMsg(angular_vel_);
    // RCLCPP_INFO(this->get_logger(), "lin_vel: {%f %f %f}", this->linear_vel_.getX(), this->linear_vel_.getY(), this->linear_vel_.getZ());
    // RCLCPP_INFO(this->get_logger(), "ang_vel: {%f %f %f}", this->angular_vel_.getX(), this->angular_vel_.getY(), this->angular_vel_.getZ());
    this->vel_pub_->publish(cmd_vel);
}

void TurtleBot3Controller::publish_pose()
{
    geometry_msgs::msg::TransformStamped odom2robot;

    try {
        odom2robot = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException& e) {
        RCLCPP_ERROR(this->get_logger(), "Odom to robot transform not found: %s", e.what());
        return;
    }

    geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position.x = odom2robot.transform.translation.x;
    pose.pose.position.y = odom2robot.transform.translation.y;
    pose.pose.position.z = odom2robot.transform.translation.z;
    tf2::convert(odom2robot.transform.rotation, pose.pose.orientation);

    // test
    // double roll, pitch, yaw;
    // tf2::Quaternion q;
    // tf2::convert(odom2robot.transform.rotation, q);
    // tf2::getEulerYPR(q, yaw, pitch, roll);
    // RCLCPP_INFO(this->get_logger(), "{yaw}: {%f}", yaw);
    
    // RCLCPP_INFO(this->get_logger(), "lin_vel: {%f %f %f}", this->linear_vel_.getX(), this->linear_vel_.getY(), this->linear_vel_.getZ());
    this->pose_pub_->publish(pose);
}

rcl_interfaces::msg::SetParametersResult TurtleBot3Controller::param_change_callback(const std::vector<rclcpp::Parameter> &params)
{
    // Create a result object to report the outcome of parameter changes.
    auto result = rcl_interfaces::msg::SetParametersResult();
 
    // Assume success unless an unsupported parameter is encountered.
    result.successful = true;

    // Iterate through each parameter in the change request.    
    for (const auto &param : params) 
    {
        // Check if the changed parameter is 'velocity_limit' and of type double.
        if (param.get_name() == "speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) 
        {
            this->speed_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'speed' has changed. The new value is: %f", param.as_double());
        } 
        // Handle any unsupported parameters.
        else
        {
            // Mark the result as unsuccessful and provide a reason.
            result.successful = false;
            result.reason = "Unsupported parameter";
        }
    } 
    return result;
}