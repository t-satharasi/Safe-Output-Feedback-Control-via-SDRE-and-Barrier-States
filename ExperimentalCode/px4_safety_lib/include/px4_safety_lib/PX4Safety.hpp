#ifndef PX4_SAFETY_HPP
#define PX4_SAFETY_HPP

#include <iostream>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace px4_safety_lib {
    class PX4Safety {
    private:
        std::reference_wrapper<rclcpp::Node> node_;

        std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> obs_pose_subs_;

        rclcpp::TimerBase::SharedPtr obs_viz_timer_;
        visualization_msgs::msg::MarkerArray obs_markers_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_viz_publisher_;

	    geometry_msgs::msg::Point fence_min_, fence_max_;
	    double fence_a_, fence_b_, fence_p_;
	    double obs_a_, obs_b_, obs_p_;

        double rho_min_fence_, rho_min_obs_;
        double max_influence_;

        std::vector<std::string> obstacles_;
        geometry_msgs::msg::PoseArray obs_poses_;

        bool enable_viz_;

        geometry_msgs::msg::Point normalize_vector(geometry_msgs::msg::Point vector_in);
        double vector_norm(geometry_msgs::msg::Point vector_in);
        double euclidean_distance(geometry_msgs::msg::Point vector0, geometry_msgs::msg::Point vector1);
        geometry_msgs::msg::Point vector_sum(geometry_msgs::msg::Point vector_0, geometry_msgs::msg::Point vector_1);
        double fence_influence_function(double dist);
        double obs_influence_function(double dist);

        void obstacle_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg, int obs_id);
        void publish_obs_viz();


        // init functions
        void initialize();
        void init_parameters();
        void visualize_obstacles();

        // helper to get node reference
        inline rclcpp::Node& node() { return node_.get(); }

    public:
    	PX4Safety(rclcpp::Node &parent_node);
        geometry_msgs::msg::Twist compute_safe_cmd_vel(
            geometry_msgs::msg::Pose agent_pose,
            geometry_msgs::msg::Twist cmd_vel_in
        );
    };
}

#endif
