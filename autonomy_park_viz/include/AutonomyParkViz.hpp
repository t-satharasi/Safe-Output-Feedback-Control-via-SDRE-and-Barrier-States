#ifndef AUTONOMY_PARK_VIZ_HPP
#define AUTONOMY_PARK_VIZ_HPP

//C++ libraries
#include <chrono>
#include <memory>

//ROS2
#include <rclcpp/rclcpp.hpp>

//TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//Message types
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class AutonomyParkViz : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;

    //Polygons
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr low_border_publisher_, high_border_publisher_;
    geometry_msgs::msg::PolygonStamped low_border_poly_msg_, high_border_poly_msg_;

    //Rviz markers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr posts_publisher_;
    visualization_msgs::msg::MarkerArray post_markers_;
    // std::vector<geometry_msgs::msg::Point> post_positions_;
    // geometry_msgs::msg::Vector3 post_scale_;
    // std_msgs::msg::ColorRGBA post_color_;
    // std_msgs::msg::ColorRGBA near_post_color_;

    int num_subscribers_;

    void timer_callback();
public:
    AutonomyParkViz();
};

#endif