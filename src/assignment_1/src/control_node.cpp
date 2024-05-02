#include "control_node.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>
#include <angles/angles.h>


using namespace std::chrono_literals;
using namespace std::placeholders;

TurtleBot3Controller::TurtleBot3Controller(const rclcpp::NodeOptions & options, const std::string & node_name)
    : Node(node_name, options), tf_buffer_(), tf_listener_(tf_buffer_)
{
    rcl_interfaces::msg::ParameterDescriptor speed_descriptor;
    speed_descriptor.description = "Constant speed for the TurtleBot3";

    this->declare_parameter<double>("speed", DEFAULT_SPEED, speed_descriptor);
    this->set_parameters();

    linear_vel_ = {0, 0, 0};
    angular_vel_ = {0, 0, 0};

    // initialize the setpoints for the square
    this->init_setpoints();

    goal_ = setpoints_.front();
    goal_success_ = false;

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TurtleBot3Controller::param_change_callback, this, std::placeholders::_1));
    
    // service for default speed setting
    this->speed_service_ = this->create_service<speed_interface::srv::SetSpeed>("set_speed", std::bind(&TurtleBot3Controller::set_speed, this, _1));

    this->timer_ = this->create_wall_timer(15ms, std::bind(&TurtleBot3Controller::go_in_square, this));

    // 6D position publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);

    // Control velocity publisher
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    this->vel_timer_ = this->create_wall_timer(10ms, std::bind(&TurtleBot3Controller::send_velocity, this));
    this->pose_timer_ = this->create_wall_timer(150ms, std::bind(&TurtleBot3Controller::publish_pose, this));
}

void TurtleBot3Controller::set_parameters()
{
    this->speed_ = this->get_parameter("speed").as_double();
}

void TurtleBot3Controller::set_speed(const std::shared_ptr<speed_interface::srv::SetSpeed::Request> request)
{
    if (request->speed >= MAX_SPEED)
    {
        RCLCPP_WARN(this->get_logger(), "Setting maximum supported speed: %f. Consider switching to lower speed to avoid operating your robot at critical conditions.", MAX_SPEED);
        RCLCPP_WARN(this->get_logger(), "Robot might behave in unstable manner. Consider increasing the tolerances if you want to run at maximum speed.");
        this->speed_ = MAX_SPEED;
    }
    else
        this->speed_ = request->speed;
    RCLCPP_INFO(this->get_logger(), "Set speed to: {%f}", this->speed_);
}

// used for setpoints initialization
double TurtleBot3Controller::normalize_angle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

void TurtleBot3Controller::init_setpoints()
{
    rclcpp::Rate rate(1s);

    geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr;
    // simulated robot is not spawned yet - wait
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

    setpoints_.push_back({x_origin+5, y_origin, yaw_origin});
    setpoints_.push_back({x_origin+5, y_origin+5, normalize_angle(yaw_origin+M_PI_2)});
    setpoints_.push_back({x_origin, y_origin+5, normalize_angle(yaw_origin+2*M_PI_2)});
    setpoints_.push_back({x_origin, y_origin, normalize_angle(yaw_origin+3*M_PI_2)});
    setpoints_.shrink_to_fit();
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
    return std::make_shared<geometry_msgs::msg::TransformStamped>(std::move(odom2robot));
}

double TurtleBot3Controller::get_linear_velocity(double x, double y) {
    double vel_x = 0;
    double dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    RCLCPP_DEBUG(this->get_logger(), "Linear distance: %f", dist);

    if (abs(dist) > distanceTolerance) {
        vel_x = K_l * dist;
        if (abs(vel_x) > this->speed_)
            vel_x = (dist > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_x;
}

// heading angle control
double TurtleBot3Controller::get_angular_velocity(double theta) {
    double vel_theta = 0;
    // double theta = atan2(sin(yaw), cos(yaw));

    RCLCPP_DEBUG(this->get_logger(), "Angular distance: %f", theta);

    if (abs(theta) > headingAngleTolerance) {
        vel_theta = K_ha * theta;
        if (abs(vel_theta) > this->speed_)
            vel_theta = (theta > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_theta;
}

// turning angle control
double TurtleBot3Controller::get_angular_turn_velocity(double theta) {
    double vel_theta = 0;
    // double theta = atan2(sin(yaw), cos(yaw));

    RCLCPP_DEBUG(this->get_logger(), "Angular distance: %f", theta);

    if (abs(theta) > turnAngleTolerance) {
        vel_theta = K_ta * theta;
        if (abs(vel_theta) > this->speed_)
            vel_theta = (theta > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_theta;
}

void TurtleBot3Controller::turn_control_cycle()
{
    static double old_theta = 0;

    auto odom2robot_ptr = this->get_position();
    if (odom2robot_ptr == nullptr)
        return;
    auto odom2robot = *(odom2robot_ptr.get());

    orientation_.setX(odom2robot.transform.rotation.x);
    orientation_.setY(odom2robot.transform.rotation.y);
    orientation_.setZ(odom2robot.transform.rotation.z);
    orientation_.setW(odom2robot.transform.rotation.w);
    
    double yaw;
    yaw = tf2::getYaw(orientation_);

    RCLCPP_DEBUG(this->get_logger(), "Yaw: {%f}", yaw);
    RCLCPP_DEBUG(this->get_logger(), "Desired yaw: {%f}", goal_.yaw);

    double yaw_diff = angles::shortest_angular_distance(yaw, goal_.yaw);

    double vel_theta = get_angular_turn_velocity(yaw_diff);
    old_theta = vel_theta;
    angular_vel_.setZ(vel_theta);
    // applying tiny back acceleration burst to weaken simulated robot's spontaneous drift after turning
    if (vel_theta == 0) {
        angular_vel_.setZ(-1*old_theta*K_ta);
        send_velocity();
        angular_vel_.setZ(0);
        send_velocity();
        angle_goal_success_ = true;
    }
}

void TurtleBot3Controller::control_cycle()
{
    static double old_vel = 0;
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
    
    RCLCPP_DEBUG(this->get_logger(), "x,y: {%f %f}", x, y);
    RCLCPP_DEBUG(this->get_logger(), "Desired position: {%f %f}", goal_.x, goal_.y);

    double x_diff = goal_.x - x;
    double y_diff = goal_.y - y;
    double yaw_desired = atan2(y_diff, x_diff);
    double yaw_diff = angles::shortest_angular_distance(yaw, yaw_desired);

    double vel_x = get_linear_velocity(x_diff, y_diff);
    old_vel = vel_x;
    linear_vel_.setX(vel_x);
    if (vel_x == 0)
    {
        angular_vel_.setZ(0);
        timer_->cancel();
        // applying tiny back acceleration burst to weaken simulated robot's spontaneous drift after turning
        linear_vel_.setX(-1*old_vel*K_l);
        send_velocity();
        linear_vel_.setX(0);
        send_velocity();
        goal_success_ = true;
        timer_->reset();
    }
    else
    {
        double vel_theta = get_angular_velocity(yaw_diff);
        angular_vel_.setZ(vel_theta);
    }
}

void TurtleBot3Controller::go_in_square()
{
    if (this->speed_ == 0)
        return;
    static auto iter = setpoints_.begin();

    if (goal_success_)
    {
        iter++;
        if (iter == setpoints_.end())
            iter = setpoints_.begin();
        goal_ = *iter;
        angle_goal_success_  = false;
        goal_success_ = false;

        RCLCPP_DEBUG(this->get_logger(), "GOAL SUCCESS! Next goal: {%f %f}", goal_.x, goal_.y);
    }
    else if (!angle_goal_success_) {
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(400), std::bind(&TurtleBot3Controller::go_in_square, this));

        turn_control_cycle();
    }
    else {
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(15), std::bind(&TurtleBot3Controller::go_in_square, this));
        control_cycle();
    }
}

void TurtleBot3Controller::send_velocity()
{
    geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear = tf2::toMsg(linear_vel_);
    cmd_vel.angular = tf2::toMsg(angular_vel_);
    this->vel_pub_->publish(cmd_vel);
}

// publish current pose relative to the robot's /odom frame of reference (starting position)
void TurtleBot3Controller::publish_pose()
{
    geometry_msgs::msg::TransformStamped odom2robot;

    try {
        odom2robot = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException& e) {
        RCLCPP_WARN(this->get_logger(), "Odom to robot transform not found: %s", e.what());
        return;
    }

    geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position.x = odom2robot.transform.translation.x;
    pose.pose.position.y = odom2robot.transform.translation.y;
    pose.pose.position.z = odom2robot.transform.translation.z;
    tf2::convert(odom2robot.transform.rotation, pose.pose.orientation);

    this->pose_pub_->publish(pose);
}

// callback for setting default speed via a ROS parameter
rcl_interfaces::msg::SetParametersResult TurtleBot3Controller::param_change_callback(const std::vector<rclcpp::Parameter> &params)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &param : params) 
    {
        // checking if the changed parameter is of the right type
        if (param.get_name() == "speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) 
        {
            if (param.as_double() >= MAX_SPEED)
            {
                RCLCPP_WARN(this->get_logger(), "Setting maximum supported speed: %f. Consider switching to lower speed to avoid operating your robot at critical conditions.", param.as_double());
                RCLCPP_WARN(this->get_logger(), "Robot might behave in unstable manner. Consider increasing the tolerances if you want to run at maximum speed.");
                this->speed_ = MAX_SPEED;
            }
            else {
                this->speed_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter 'speed' has changed. The new value is: %f", param.as_double());
            }
        } 
        else
        {
            result.successful = false;
            result.reason = "Unsupported parameter";
        }
    } 
    return result;
}