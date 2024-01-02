#include "adaptive_cruise_control/cruise_control_node.h"

AdaptiveCruiseControl::AdaptiveCruiseControl() : Node("adaptive_cruise_control") {

}

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveCruiseControl>());
    rclcpp::shutdown();
  return 0;
}
