//
// Created by turtlehouse on 02.01.24.
//

#ifndef SRC_CRUISE_CONTROL_NODE_H
#define SRC_CRUISE_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"
//#include "irobot_create_msgs/msg/"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using lidar_scan = sensor_msgs::msg::LaserScan;
using twist_msg = geometry_msgs::msg::Twist;

class AdaptiveCruiseControl : public rclcpp::Node {
public:
    AdaptiveCruiseControl();
private:

    int get_lidar_data(lidar_scan::SharedPtr scan_data);
    rclcpp::Subscription<lidar_scan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<twist_msg>::SharedPtr cmd_publisher_;

    double prev_dist;
    double curr_dist;

    double own_speed;
};

#endif //SRC_CRUISE_CONTROL_NODE_H
