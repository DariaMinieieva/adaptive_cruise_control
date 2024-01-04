#include "adaptive_cruise_control/cruise_control_node.h"

AdaptiveCruiseControl::AdaptiveCruiseControl() : Node("adaptive_cruise_control"),
                                                 desired_speed{0.5},
                                                 prev_dist{0},
                                                 curr_dist{0},
                                                 own_speed{desired_speed},
                                                 safe_distance{2},
                                                 critical_distance{0.4}
{
    lidar_subscriber_ = this->create_subscription<lidar_scan>("/scan",
                                                                               rclcpp::SensorDataQoS(),
                                                                               std::bind(&AdaptiveCruiseControl::get_lidar_data, this, std::placeholders::_1));

    odom_subscriber_ = this->create_subscription<odom_msg>("/odom",
                                                              rclcpp::SensorDataQoS(),
                                                              std::bind(&AdaptiveCruiseControl::get_odom_data, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<twist_msg>("/cmd_vel",
                                                              rclcpp::SensorDataQoS());
}

int AdaptiveCruiseControl::maintain_distance(lidar_scan::SharedPtr scan_data) {
    auto min_range = scan_data->range_min;
//    auto max_range = scan_data->range_max;

    auto ranges = scan_data->ranges;

    auto cmd_msg = twist_msg();

    double cum_dist = 0;
    size_t count_dist = 0;

    for (size_t i = 7*ranges.size()/36; i < 11*ranges.size()/36; i++) {
        if (ranges[i] >= min_range && ranges[i] <= 5) {
            cum_dist += ranges[i];
            count_dist++;
        }
    }

    double distance = count_dist ? (cum_dist / static_cast<double>(count_dist)) : std::numeric_limits<double>::max();
    RCLCPP_INFO(this->get_logger(), "distance: %f m", distance);

    if (count_dist != 0 && distance < safe_distance && own_speed > 0) {
        own_speed -= 0.05;
    } else if (own_speed < desired_speed){
        own_speed += 0.05;
    }

    RCLCPP_INFO(this->get_logger(), "speed: %f m/s", own_speed);


    cmd_msg.linear.x = own_speed;
//    cmd_msg.angular.z = own_speed;
    cmd_publisher_->publish(cmd_msg);

    return 0;
}

int AdaptiveCruiseControl::get_lidar_data(const lidar_scan::SharedPtr scan_data) {
    auto time_between_scans = scan_data->scan_time;

    auto min_range = scan_data->range_min;
//    auto max_range = scan_data->range_max;

    auto ranges = scan_data->ranges;

    auto cmd_msg = twist_msg();

    double cum_dist = 0;
    size_t count_dist = 0;

    for (size_t i = 7*ranges.size()/36; i < 11*ranges.size()/36; i++) {
        if (ranges[i] >= min_range && ranges[i] <= 5) {
            cum_dist += ranges[i];
            count_dist++;
        }
    }

    double rel_speed = 0;

    double distance = count_dist ? (cum_dist / static_cast<double>(count_dist)) : std::numeric_limits<double>::max();
    RCLCPP_INFO(this->get_logger(), "distance: %f m", distance);

    if (prev_dist < 0.000001 && count_dist != 0) {

        prev_dist = distance;
    } else if (count_dist != 0) {

        curr_dist = distance;
        rel_speed = (curr_dist-prev_dist ) / time_between_scans;
        prev_dist = curr_dist;
    } else {

        curr_dist = 0;
        prev_dist = 0;
    }

    RCLCPP_INFO(this->get_logger(), "relative speed: %f m/s", rel_speed);

    if (distance >= safe_distance || (rel_speed + own_speed - desired_speed) > 0.001) {
        own_speed = desired_speed;
    } else if (distance <= critical_distance || rel_speed+own_speed < 0) {
        own_speed = 0;
        // this means the car is going on us ^)
    } else {
        own_speed += rel_speed;
    }

    cmd_msg.linear.x = own_speed;
//    cmd_msg.angular.z = own_speed;
    cmd_publisher_->publish(cmd_msg);

    RCLCPP_INFO(this->get_logger(), "Speed: %f m/s", own_speed);

    return 0;
}

int AdaptiveCruiseControl::get_odom_data([[maybe_unused]] odom_msg::SharedPtr odom_data) {
//    RCLCPP_INFO(this->get_logger(), "Estimated vel %f", odom_data->twist.twist.linear.x);
    return 0;

}

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveCruiseControl>());
    rclcpp::shutdown();
    return 0;
}
