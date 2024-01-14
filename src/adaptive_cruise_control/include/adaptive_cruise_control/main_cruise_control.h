//
// Created by turtlehouse on 13.01.24.
//

#ifndef ADAPTIVE_CRUISE_CONTROL_MAIN_CRUISE_CONTROL_H
#define ADAPTIVE_CRUISE_CONTROL_MAIN_CRUISE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "service_speed_interfaces/srv/velocities.hpp"

using speed_srv = service_speed_interfaces::srv::Velocities;

class ACC : public rclcpp::Node {
public:
    ACC();

private:
    void combine_velocities();
    void response_callback(rclcpp::Client<speed_srv>::SharedFuture future);

    rclcpp::Client<speed_srv>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif //ADAPTIVE_CRUISE_CONTROL_MAIN_CRUISE_CONTROL_H
