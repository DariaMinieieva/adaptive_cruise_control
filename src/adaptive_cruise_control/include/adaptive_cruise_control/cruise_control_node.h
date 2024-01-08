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
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"

using lidar_scan = sensor_msgs::msg::LaserScan;
using twist_msg = geometry_msgs::msg::Twist;
using odom_msg = nav_msgs::msg::Odometry;
using ir_msg = irobot_create_msgs::msg::IrIntensityVector;

struct RelativeSpeedValue {
    double prev_dist = 0;
    double curr_dist = 0;
    double distance_dbg = 0;

    double rel_speed = 0;
};

class AdaptiveCruiseControl : public rclcpp::Node {
public:
    AdaptiveCruiseControl();
private:

    int get_lidar_data(lidar_scan::SharedPtr scan_data);
    int get_odom_data(odom_msg::SharedPtr odom_data);
    int get_ir_data(ir_msg::SharedPtr ir_data);

    void get_one_side_speed(RelativeSpeedValue& speed_value, int left_side, int right_side, const lidar_scan::SharedPtr scan_data);

    rclcpp::Subscription<lidar_scan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<odom_msg>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<ir_msg>::SharedPtr ir_subscriber_;
    rclcpp::Publisher<twist_msg>::SharedPtr cmd_publisher_;

    double desired_speed;

    double own_speed;
    double safe_distance;
    double critical_distance;

    double odom_speed;

    RelativeSpeedValue left_side_view;
    RelativeSpeedValue right_side_view;
    RelativeSpeedValue center_view;
};

#endif //SRC_CRUISE_CONTROL_NODE_H
