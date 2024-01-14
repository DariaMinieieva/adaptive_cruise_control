//
// Created by turtlehouse on 02.01.24.
//

#ifndef SRC_CRUISE_CONTROL_NODE_H
#define SRC_CRUISE_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "service_speed_interfaces/srv/velocities.hpp"

using lidar_scan = sensor_msgs::msg::LaserScan;
using twist_msg = geometry_msgs::msg::Twist;
using odom_msg = nav_msgs::msg::Odometry;
using speed_srv = service_speed_interfaces::srv::Velocities;

struct RelativeSpeedValue {
    double prev_dist = 0;
    double curr_dist = 0;
    double distance_dbg = 0;

    double rel_speed = 0;
};

class LidarSpeed : public rclcpp::Node {
public:
    LidarSpeed();


private:

    int get_lidar_data(lidar_scan::SharedPtr scan_data);
    int get_odom_data(odom_msg::SharedPtr odom_data);

    void get_one_side_speed(RelativeSpeedValue& speed_value, int left_side, int right_side, const lidar_scan::SharedPtr scan_data);
    void get_speed_callback( rclcpp::Client<speed_srv>::SharedFuture future);



    rclcpp::Subscription<lidar_scan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<odom_msg>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<twist_msg>::SharedPtr cmd_publisher_;
    rclcpp::Client<speed_srv>::SharedPtr client_;



    double desired_speed;

    double own_speed;
    double safe_distance;
    double critical_distance;

    double odom_speed;

    double speed_camera_lin;
    double speed_camera_ang;

    bool new_data;

    int sfd;


    RelativeSpeedValue left_side_view;
    RelativeSpeedValue right_side_view;
    RelativeSpeedValue center_view;
};

#endif //SRC_CRUISE_CONTROL_NODE_H
