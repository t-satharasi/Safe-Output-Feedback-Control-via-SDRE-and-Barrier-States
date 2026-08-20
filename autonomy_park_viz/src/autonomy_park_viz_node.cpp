#include "AutonomyParkViz.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomyParkViz>());
    rclcpp::shutdown();
    return 0;
}

