#ifndef PX4_TELEMETRY_HPP
#define PX4_TELEMETRY_HPP

#include <rclcpp/rclcpp.hpp>

//TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geodesy/utm.h>
#include <GeographicLib/Geoid.hpp>

//Message types
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joy.hpp>

//Mavros message and service types 
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

class PX4Telemetry : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_lpos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_gpos_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr apark_pose_publisher_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr gp_origin_publisher_;

    //Declare service clients for mode, arming and takeoff/landing
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_, land_client_;

    geometry_msgs::msg::PoseStamped apark_pose_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> apark_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped apark_tf_;

    tf2::Quaternion q_utm_to_apark_, q_apark_to_utm_;

    std::string px4_id_;
    double origin_x_, origin_y_, origin_r_;
    uint8_t utm_zone_;
    char utm_band_;

    /**
    * @brief Geoid dataset used to convert between AMSL and WGS-84
    *
    * That class loads egm96_5 dataset to RAM, it is about 24 MiB.
    */
    std::shared_ptr<GeographicLib::Geoid> egm96_5_;

    mavros_msgs::msg::State current_state_;

    struct Button {
        Button() : button(0) {}
        int button;
    };

    struct {
        Button arm;
        Button disarm;
        Button offboard;
        Button follow;
        Button control;
    } buttons_;

    //Button state for debouncing
    struct ButtonState {
        ButtonState() : state(0) {}
        int state;
    };

    struct {
        ButtonState arm;
        ButtonState disarm;
        ButtonState offboard;
        ButtonState follow;
        ButtonState control;
    } button_state_;

    enum LandedState {
        undefined = 0,
        on_ground,
        in_air,
        takeoff,
        landing
    };

    LandedState landed_state_;

    bool landing_requested_;
    std::string loiter_str_, offboard_str_;
    double altitude_amsl_;
    double battery_voltage_;

    //Track initialization of various messages
    bool alt_init_, lpos_init_, gpos_init_;

    bool sim_mode_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

    void state_callback(const mavros_msgs::msg::State::SharedPtr state_msg);
    void ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg);
    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg);
    void global_lpos_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void global_gpos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    
    int get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button);
    void send_arming_request(bool arm);
    void send_tol_request(bool takeoff);

    //Service response callbacks
    void loiter_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void offboard_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
    void tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future);

    //Utility functions
    geographic_msgs::msg::GeoPose apark_to_global(const geometry_msgs::msg::Pose &apark_pose);
    double quat_to_yaw(geometry_msgs::msg::Quaternion quat);

public:
    PX4Telemetry();
};

#endif


