#include "PX4Telemetry.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4Telemetry>());
    rclcpp::shutdown();
    return 0;
}