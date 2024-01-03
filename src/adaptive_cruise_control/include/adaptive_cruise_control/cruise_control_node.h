//
// Created by turtlehouse on 02.01.24.
//

#ifndef SRC_CRUISE_CONTROL_NODE_H
#define SRC_CRUISE_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"
//#include "irobot_create_msgs/msg/"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using lidar_scan = sensor_msgs::msg::LaserScan;
using twist_msg = geometry_msgs::msg::Twist;
using odom_msg = nav_msgs::msg::Odometry;

class AdaptiveCruiseControl : public rclcpp::Node {
public:
    AdaptiveCruiseControl();
private:

    int get_lidar_data(lidar_scan::SharedPtr scan_data);
    int get_odom_data(odom_msg::SharedPtr odom_data);
    rclcpp::Subscription<lidar_scan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<odom_msg>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<twist_msg>::SharedPtr cmd_publisher_;

    double desired_speed;

    double prev_dist;
    double curr_dist;
    double own_speed;
    double safe_distance;
    double critical_distance;
};

#endif //SRC_CRUISE_CONTROL_NODE_H
