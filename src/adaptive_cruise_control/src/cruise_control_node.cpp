#include "adaptive_cruise_control/cruise_control_node.h"

AdaptiveCruiseControl::AdaptiveCruiseControl() : Node("adaptive_cruise_control"),
                                                 prev_dist{0},
                                                 curr_dist{0},
                                                 own_speed{0.3}{
    lidar_subscriber_ = this->create_subscription<lidar_scan>("/scan",
                                                                               rclcpp::SensorDataQoS(),
                                                                               std::bind(&AdaptiveCruiseControl::get_lidar_data, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<twist_msg>("/cmd_vel",
                                                              rclcpp::SensorDataQoS());
}

int AdaptiveCruiseControl::get_lidar_data(const lidar_scan::SharedPtr scan_data) {
    auto time_sec = scan_data->header.stamp.sec;
    auto time_nanosec = scan_data->header.stamp.nanosec;

    auto min_angle = scan_data->angle_min;
    auto max_angle = scan_data->angle_max;
    auto ang_dist_measurments = scan_data->angle_increment;

    auto time_between_measurements = scan_data->time_increment;

    auto time_between_scans = scan_data->scan_time;

    auto min_range = scan_data->range_min;
    auto max_range = scan_data->range_max;

    auto ranges = scan_data->ranges;

    auto intensities = scan_data->intensities;

//    RCLCPP_INFO(this->get_logger(), "Time: %i:%i", time_sec, time_nanosec);
//    RCLCPP_INFO(this->get_logger(), "Angle min-max: %f-%f", min_angle, max_angle);
//    RCLCPP_INFO(this->get_logger(), "Angular distance between measurements: %f",  ang_dist_measurments);
//    RCLCPP_INFO(this->get_logger(), "Time between measurements and scans: %f %f", time_between_measurements, time_between_scans);
//    RCLCPP_INFO(this->get_logger(), "Range min max: %f-%f", min_range, max_range);

    double cum_dist = 0;
    size_t count_dist = 0;

    for (size_t i = 2*ranges.size()/9; i < 5*ranges.size()/18; i++) {
        if (ranges[i] >= min_range && ranges[i] <= 2) {
//            RCLCPP_INFO(this->get_logger(), "Range: %zu %f", i, ranges[i]);
            cum_dist += ranges[i];
            count_dist++;
        }
    }

    double speed = 0;

    if (prev_dist == 0 && count_dist != 0) {
        prev_dist = cum_dist / static_cast<double>(count_dist);

        return 0;
    } else if (count_dist != 0) {
        curr_dist = cum_dist / static_cast<double>(count_dist);
        speed = (curr_dist-prev_dist ) / time_between_scans;

        prev_dist = curr_dist;
    }

    auto cmd_msg = twist_msg();



    speed += own_speed;
    RCLCPP_INFO(this->get_logger(), "own speed: %f m/s", own_speed);


    if (speed > 0.3) {
        speed = 0.3;
    } else if (speed < 0) {
        speed = 0;
    }

    own_speed = speed;
    cmd_msg.linear.x = speed;

    cmd_publisher_->publish(cmd_msg);

    RCLCPP_INFO(this->get_logger(), "Speed: %f m/s", speed);


    return 0;
}

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveCruiseControl>());
    rclcpp::shutdown();
    return 0;
}
