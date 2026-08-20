#include "PX4Teleop.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PX4Teleop::PX4Teleop() : Node("px4_teleop_node"), pose_init_(false) {
	RCLCPP_INFO(this->get_logger(), "Initializing PX4 Teleop Node");

    //Get my namespace (remove the slash with substr)
    agent_id_ = std::string(this->get_namespace()).substr(1);

    //Get controller parameters
    this->declare_parameter("x_axis", -1);
    this->declare_parameter("y_axis", -1);
    this->declare_parameter("z_axis", -1);
    this->declare_parameter("yaw_axis", -1);

    this->declare_parameter("x_vel_max", 1.0);
    this->declare_parameter("y_vel_max", 1.0);
    this->declare_parameter("z_vel_max", 1.0);
    this->declare_parameter("yaw_vel_max", 1.0);

    this->get_parameter("x_axis", axes_.x.axis);
    this->get_parameter("y_axis", axes_.y.axis);
    this->get_parameter("z_axis", axes_.z.axis);
    this->get_parameter("yaw_axis", axes_.yaw.axis);

    this->get_parameter("x_vel_max", axes_.x.factor);
    this->get_parameter("y_vel_max", axes_.y.factor);
    this->get_parameter("z_vel_max", axes_.z.factor);
    this->get_parameter("yaw_vel_max", axes_.yaw.factor);

    RCLCPP_INFO(this->get_logger(), "Loaded controller axis parameters:\nX: %d, Y: %d, Z: %d, Yaw: %d", axes_.x.axis, axes_.y.axis, axes_.z.axis, axes_.yaw.axis);

    //Get park rotation from params (not optimal but needed because vel commands go to global ENU frame)
    this->declare_parameter("origin_r", 0.0);
    if (this->get_parameter("origin_r", origin_r_)) {
        RCLCPP_INFO(this->get_logger(), "Park origin rotation set to %.4f rad.", origin_r_);
        cos_origin_ = cos(origin_r_);
        sin_origin_ = sin(origin_r_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Park origin rotation not set. Exiting.");
        rclcpp::shutdown();
        return;
    }

    // initialize safety
	try {
        px4_safety = std::make_unique<px4_safety_lib::PX4Safety>(*this);
	} catch (const std::runtime_error &e) {
		RCLCPP_FATAL(this->get_logger(), "%s", e.what());
		throw;
	}

    setpoint_vel_.header.frame_id = agent_id_;

    //Publish setpoints at 50Hz
    setpoint_timer_ = this->create_wall_timer(20ms, std::bind(&PX4Teleop::publish_setpoint, this));
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("setpoint_velocity/cmd_vel", 10);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PX4Teleop::joy_callback, this, _1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("autonomy_park/pose", 10, std::bind(&PX4Teleop::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "PX4 Teleop Initialized.");
}

void PX4Teleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    if (!pose_init_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring joy inputs until agent pose is initialized.");
        return;
    }

    double apark_vx = get_axis(joy_msg, axes_.x);
    double apark_vy = get_axis(joy_msg, axes_.y);

    geometry_msgs::msg::Twist unsafe_cmd_vel;
    unsafe_cmd_vel.linear.x = apark_vx;
    unsafe_cmd_vel.linear.y = apark_vy;
    unsafe_cmd_vel.linear.z = get_axis(joy_msg, axes_.z);
    unsafe_cmd_vel.angular.z = get_axis(joy_msg, axes_.yaw);

    //Generate safe velocity command
    geometry_msgs::msg::Twist safe_cmd_vel = px4_safety->compute_safe_cmd_vel(agent_pose_, unsafe_cmd_vel);

    //Convert autonomy park X/Y velocity command to ENU frame
    setpoint_vel_.twist.linear.x = cos_origin_*safe_cmd_vel.linear.x + sin_origin_*safe_cmd_vel.linear.y;
    setpoint_vel_.twist.linear.y = -sin_origin_*safe_cmd_vel.linear.x + cos_origin_*safe_cmd_vel.linear.y;
    setpoint_vel_.twist.linear.z = safe_cmd_vel.linear.z;
    setpoint_vel_.twist.angular.z = safe_cmd_vel.angular.z;
}

void PX4Teleop::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    //Update PX4 agent pose
    agent_pose_ = pose_msg->pose;

    if (!pose_init_) pose_init_ = true;
}

void PX4Teleop::publish_setpoint() {
    rclcpp::Time now = this->get_clock()->now();
    setpoint_vel_.header.stamp = this->get_clock()->now();
    vel_publisher_->publish(setpoint_vel_);
}

double PX4Teleop::get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis) {
    if (axis.axis < 0 || std::abs(axis.axis) > (int)joy_msg->axes.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Axis %d out of range, joy has %d axes", axis.axis, (int)joy_msg->axes.size());
        return -1;
    }

    double output = joy_msg->axes[std::abs(axis.axis)] * axis.factor + axis.offset;

    return output;
}
