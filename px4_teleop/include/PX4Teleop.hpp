#ifndef PX4_TELEOP_HPP
#define PX4_TELEOP_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <px4_safety_lib/PX4Safety.hpp>

class PX4Teleop : public rclcpp::Node {
private:
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_publisher_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    rclcpp::TimerBase::SharedPtr setpoint_timer_;

    struct Axis {
        Axis() : axis(0), factor(0.0), offset(0.0) {}

        int axis;
        double factor;
        double offset;
    };

    struct {
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
    } axes_;

    enum LandedState {
        undefined = 0,
        on_ground,
        in_air,
        takeoff,
        landing
    };

    LandedState landed_state_;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::Pose agent_pose_;

    std::string agent_id_;
    geometry_msgs::msg::TwistStamped setpoint_vel_;

    double origin_r_, cos_origin_, sin_origin_;

    bool pose_init_;
    std::unique_ptr<px4_safety_lib::PX4Safety> px4_safety;

	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);

	void state_callback(const mavros_msgs::msg::State::SharedPtr state_msg);
	void ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg);

	void publish_setpoint();

    //Joy
    double get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis);
public:
    PX4Teleop();
};

#endif
