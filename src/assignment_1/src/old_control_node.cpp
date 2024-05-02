#include "control_node.hpp"
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

    this->state = REST;

    linear_vel_ = {0, 0, 0};
    angular_vel_ = {0, 0, 0};

    // this->timer_ = this->create_wall_timer(std::chrono::milliseconds(180), std::bind(&TurtleBot3Controller::control_cycle, this));
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&TurtleBot3Controller::control_cycle, this));

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

// double TurtleBot3Controller::get_euclidian_distance(double x, double y)
// {
//     return abs(std::sqrt(std::pow(x-this->x_init_, 2) + std::pow(y-this->y_init_, 2)) - SQUARE_POLYGON);
// }

double TurtleBot3Controller::get_euclidian_distance(double x, double y)
{
    return (SQUARE_POLYGON - std::sqrt(std::pow(x-this->x_init_, 2) + std::pow(y-this->y_init_, 2)));
}

double TurtleBot3Controller::set_linear_velocity(double x, double y) {
    double vel_x = 0;
    double dist = this->get_euclidian_distance(x, y);
    RCLCPP_INFO(this->get_logger(), "linear dist: %f", dist);

    if (abs(dist) > distanceTolerance) {
        // The magnitude of the robot's velocity is directly
        // proportional to the distance the robot is from the 
        // goal.
        vel_x = K_l * dist;
        if (abs(vel_x) > this->speed_)
            vel_x = (dist > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_x;
}

// double TurtleBot3Controller::set_linear_velocity(double x, double y) {
//     double vel_x = 0;
//     double dist = this->get_euclidian_distance(x, y);
//     RCLCPP_INFO(this->get_logger(), "linear dist: %f", dist);

//     if (dist > distanceTolerance) {
//         // The magnitude of the robot's velocity is directly
//         // proportional to the distance the robot is from the 
//         // goal.
//         vel_x = K_l * dist;
//         vel_x = (vel_x > this->speed_)? this->speed_ : vel_x;
//     }
//     return vel_x;
// }

double normalizeAngle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

double TurtleBot3Controller::get_angular_distance(double yaw)
{
    // Normalize current_yaw to [-π, π] range

    // Calculate the new yaw angle after rotation
    // double goal_yaw = yaw_init_ + M_PI_2;
    // RCLCPP_INFO(this->get_logger(), "yaw_init_: %f, yaw_init_+(pi/2): %f", yaw_init_, new_yaw);

    // Normalize the new_yaw to [-π, π] range
    // goal_yaw = normalizeAngle(goal_yaw);

    // Set the new yaw angle
    // RCLCPP_INFO(this->get_logger(), "yaw: %f, new_yaw: %f, yaw-new_yaw: %f", yaw, new_yaw, yaw-new_yaw);

    return abs(yaw - goal_yaw_);
}

double TurtleBot3Controller::get_angular_distance2(double yaw)
{
    yaw = angles::to_degrees(angles::normalize_angle_positive(yaw));
    double normalized_goal_yaw = angles::to_degrees(angles::normalize_angle_positive(goal_yaw_)); // TODO: calculate beforehand
        RCLCPP_INFO(this->get_logger(), "yaw: %f, goal_yaw: %f, yaw-goal_yaw: %f", yaw, normalized_goal_yaw, yaw-normalized_goal_yaw);

    return normalized_goal_yaw - yaw;
}

double TurtleBot3Controller::get_angular_distance3(double yaw)
{
    double shortest_dist = angles::shortest_angular_distance(yaw, goal_yaw_);
    
    RCLCPP_INFO(this->get_logger(), "yaw: %f, goal_yaw: %f, shortest dist: %f", yaw, goal_yaw_, shortest_dist);

    return shortest_dist;
}

double TurtleBot3Controller::set_angular_velocity(double yaw) {
    double vel_theta = 0;
    double theta = this->get_angular_distance3(yaw);

    if (abs(theta) > angleTolerance) {
        vel_theta = K_a * theta;
        if (abs(vel_theta) > this->speed_)
            vel_theta = (theta > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_theta;
}

// latest
// double TurtleBot3Controller::set_angular_velocity(double yaw) {
//     double vel_theta = 0;
//     double theta = this->get_angular_distance2(yaw);
//     double theta_rad = angles::from_degrees(theta);
//     RCLCPP_INFO(this->get_logger(), "angular dist: %f", theta);
//     RCLCPP_INFO(this->get_logger(), "angular dist in rad: %f", theta_rad);

//     if (abs(theta) > angleTolerance) {
//         vel_theta = K_a * theta_rad;
//         if (abs(vel_theta) > this->speed_)
//             vel_theta = (theta > 0)? this->speed_ : this->speed_*-1;
//     }
//     return vel_theta;
// }

// double TurtleBot3Controller::set_angular_velocity(double yaw) {
//     double vel_theta = 0;
//     double theta = this->get_angular_distance2(yaw);
//     RCLCPP_INFO(this->get_logger(), "angular dist: %f", theta);

//     if (abs(theta) > angleTolerance) {
//         vel_theta = K_a * theta;
//         if (abs(vel_theta) > this->speed_)
//             vel_theta = (theta > 0)? this->speed_ : this->speed_*-1;
//     }
//     return vel_theta;
// }

// double TurtleBot3Controller::set_angular_velocity(double yaw) {
//     double vel_theta = 0;
//     double theta = this->get_angular_distance(yaw);
//     RCLCPP_INFO(this->get_logger(), "angular dist: %f", theta);

//     if (theta > angleTolerance) {
//         vel_theta = K_a * theta;
//         vel_theta = (vel_theta > this->speed_)? this->speed_ : vel_theta;
//     }
//     return vel_theta;
// }

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
    // double z = odom2robot.transform.translation.z;

    orientation_.setX(odom2robot.transform.rotation.x);
    orientation_.setY(odom2robot.transform.rotation.y);
    orientation_.setZ(odom2robot.transform.rotation.z);
    orientation_.setW(odom2robot.transform.rotation.w);
    
    double yaw;
    yaw = tf2::getYaw(orientation_);
    // double roll, pitch, yaw;
    // tf2::getEulerYPR(orientation_, yaw, pitch, roll);

    // RCLCPP_INFO(this->get_logger(), "{x y z yaw}: {%f %f %f %f}", x, y, z, yaw);

    // rclcpp::Rate rate(5s);
    if (this->state == REST) {
        x_init_ = x;
        y_init_ = y;
        this->state = FORWARD;
    }
    else if (this->state == FORWARD) {
        double vel_x = this->set_linear_velocity(x, y);
        // RCLCPP_INFO(this->get_logger(), "setting linear velocity: %f", vel_x);
        linear_vel_.setX(vel_x);
        // send_velocity();
        if (vel_x == 0) {
            yaw_init_ = yaw;
            goal_yaw_ = yaw + M_PI_2;
            goal_yaw_ = normalizeAngle(goal_yaw_);
            this->state = TURN;
            // this->timer_->cancel();
            // rate.sleep();
            // this->timer_->reset();
        }
    }
    else if (this->state == TURN)
    {
        double vel_theta = this->set_angular_velocity(yaw);
        // RCLCPP_INFO(this->get_logger(), "setting angular velocity: %f", vel_theta);
        angular_vel_.setZ(vel_theta);
        // send_velocity();
        if (vel_theta == 0) {
            x_init_ = x;
            y_init_ = y;
            this->state = FORWARD;
            // this->timer_->cancel();
            // angular_vel_.setX(0);
            // angular_vel_.setY(0);
            // angular_vel_.setZ(0);
            // for (int i=0; i<1000; ++i)
            //     send_velocity();
            // // rate.sleep();
            // this->timer_->reset();
        }
    }
}

void TurtleBot3Controller::test_get_pose()
{
    geometry_msgs::msg::TransformStamped odom2robot;

    try {
        odom2robot = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException& e) {
        RCLCPP_ERROR(this->get_logger(), "Odom to robot transform not found: %s", e.what());
        return;
    }

    double x = odom2robot.transform.translation.x;
    double y = odom2robot.transform.translation.y;
    double z = odom2robot.transform.translation.z;

    RCLCPP_INFO(this->get_logger(), "{x y z}: {%f %f %f}", x, y, z);
}

// void TurtleBot3Controller::go_forward()
// {
//     this->angular_vel_.setZ(0.0);
//     this->linear_vel_.setX(speed_);
//     send_velocity();
//     this->linear_vel_.setX(0.0);
//     // std::chrono::duration<double> duration(5.0 / speed_);
//     // rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
// }

// void TurtleBot3Controller::turn()
// {
//     this->linear_vel_.setX(0.0);
//     this->angular_vel_.setZ(0.5);
    
//     send_velocity();
// }

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