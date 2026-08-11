#include "AutonomyParkViz.hpp"

using namespace std::chrono_literals;

AutonomyParkViz::AutonomyParkViz() : Node("autonomy_park_viz_node"), num_subscribers_(0) {
	RCLCPP_INFO(this->get_logger(), "Initializing Autonomy Park Visualization Node");

	//Get parameters
	// this->declare_parameter("border", rclcpp::PARAMETER_STRING);

	//     rclcpp::Publisher<geometry_msgs::msg::PolygonStamped> low_border_publisher_, high_border_publisher_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray> posts_publisher_;

    // this->declare_parameter("my_int", 1, rclcpp::PARAMETER_INTEGER);
    std::vector<double>temp;
    this->declare_parameter("border_x", temp);
    this->declare_parameter("border_y", temp);
    this->declare_parameter("post_x", temp);
    this->declare_parameter("post_y", temp);

    std::vector<double>border_x, border_y, post_x, post_y;
    this->get_parameter("border_x", border_x);
    this->get_parameter("border_y", border_y);
    this->get_parameter("post_x", post_x);
    this->get_parameter("post_y", post_y);

    //Enforce border size
    if (border_x.size() != 4 || border_y.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "(AutonomyParkViz) Border X and Y must be length 4.");
    }

    if (post_x.size() != 12 || post_x.size() != 12) {
        RCLCPP_ERROR(this->get_logger(), "(AutonomyParkViz) Post X and Y must be length 12.");
    }

    //Initialize polys
    low_border_poly_msg_.header.frame_id = high_border_poly_msg_.header.frame_id = "autonomy_park";

    //Set timestamps
    rclcpp::Time now = this->get_clock()->now();
    low_border_poly_msg_.header.stamp = now;
    high_border_poly_msg_.header.stamp = now;

    //Populate coordinates for border polys
    for (size_t i=0; i < border_x.size(); i++) {
        geometry_msgs::msg::Point32 p_low, p_high;

        p_low.x = p_high.x = border_x[i];
        p_low.y = p_high.y = border_y[i];
        p_low.z = 0.0;
        p_high.z = 5.0;

        low_border_poly_msg_.polygon.points.push_back(p_low);
        high_border_poly_msg_.polygon.points.push_back(p_high);
    }

    //Set marker types
    // low_border_marker.type = high_border_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // post_marker.type = visualization_msgs::msg::Marker::CYLINDER;

    // //Set colors
    // 

    // //Border is white
    // border_color.r = 1.0;
    // border_color.g = 1.0;
    // border_color.b = 1.0;
    // border_color.a = 1.0;
    // low_border_marker.color = high_border_marker.color = border_color;

    //Precompute post colors
    std_msgs::msg::ColorRGBA post_color, near_post_color;
    post_color.a = 1.0;
    post_color.r = 102.0;
    post_color.b = 0.0;
    post_color.g = 51.0;
    near_post_color.a = 1.0;
    near_post_color.r = 100.0;
    near_post_color.b = 0.0;
    near_post_color.g = 0.0;

    //Precompute post scales
    geometry_msgs::msg::Vector3 post_scale;
    post_scale.x = post_scale.y = 0.5;
    post_scale.z = 5.0;

    for (size_t i=0; i < post_x.size(); i++) {
        geometry_msgs::msg::Point p;
        p.x = post_x[i];
        p.y = post_y[i];
        p.z = 2.5;

        visualization_msgs::msg::Marker post;
        post.header.stamp = now;
        post.header.frame_id = "autonomy_park";
        post.action = visualization_msgs::msg::Marker::ADD;

        post.ns = "posts";
        post.id = i;
        post.type = visualization_msgs::msg::Marker::CYLINDER;
        post.scale = post_scale;

        if (i == 0 || i == 11) {
            post.color = near_post_color;
        } else {
            post.color = post_color;
        }

        post.pose.position = p;
        post.pose.orientation.w = 1.0;

        post_markers_.markers.push_back(post);
    }

    //Add one more point to complete the loop
    // geometry_msgs::msg::Point p_low, p_high;
    // p_low.x = p_high.x = border_x[0];
    // p_low.y = p_high.y = border_y[0];
    // p_low.z = 0.0;
    // p_high.z = 5.0;
    // low_border_marker.points.push_back(p_low);
    // high_border_marker.points.push_back(p_high);

    //Wait until someone subscribes before publishing
    // rclcpp::Rate loop_rate(1);
    // while (!posts_publisher_->get_subscription_count()) {
    //     loop_rate.sleep();
    // }
    
    // low_border_marker.header.frame_id = "autonomy_park";
    // low_border_marker.ns = "while_line";
    // low_border_marker.id = 0;
    // low_border_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // low_border_marker.action = 0;
    // low_border_marker.pose = geometry_msgs::msg::Pose();
    // low_border_marker.scale.x = 0.02;
    // low_border_marker.color.a = 1.0;
    // low_border_marker.color.b = 1.0;
    // low_border_marker.color.g = 1.0;
    // low_border_marker.color.r = 1.0;
    // auto point1 = geometry_msgs::msg::Point();
    // point1.x = 1.0;
    // point1.y = 1.0;
    // point1.z = 0.0;
    // auto point2 = geometry_msgs::msg::Point();
    // point2.x = 2.0;
    // point2.y = 2.0;
    // point2.z = 0.0;
    // low_border_marker.points.push_back(point1);
    // low_border_marker.points.push_back(point2);
    // marker_array_.markers.push_back(low_border_marker);
    // marker_array_.markers.push_back(high_border_marker);

    low_border_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("low_border", 1);
    high_border_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("high_border", 1);
    posts_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("posts", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(1s, std::bind(&AutonomyParkViz::timer_callback, this));
}

void AutonomyParkViz::timer_callback() {
    //Publish messages only for new subscribers
    // int current_num_subs = marker_publisher_->get_subscription_count();
    // if (current_num_subs > 0 && num_subscribers_ != current_num_subs) {
    //     rclcpp::Time now = this->get_clock()->now();
    //     for (size_t i=0; i < marker_array_.markers.size(); i++) {
    //         marker_array_.markers[i].header.stamp = now;
    //     }
    //     this->marker_publisher_->publish(marker_array_);

    //     num_subscribers_ = current_num_subs;
    // }

    this->low_border_publisher_->publish(low_border_poly_msg_);
    this->high_border_publisher_->publish(high_border_poly_msg_);
    this->posts_publisher_->publish(post_markers_);
}
