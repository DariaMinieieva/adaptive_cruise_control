#include "adaptive_cruise_control/cruise_control_node.h"

AdaptiveCruiseControl::AdaptiveCruiseControl() : Node("adaptive_cruise_control"),
                                                 desired_speed{0.3},
//                                                 prev_dist{0},
//                                                 curr_dist{0},
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

//    ir_subscriber_ = this->create_subscription<ir_msg>("/ir_intensity",
//                                                         rclcpp::SensorDataQoS(),
//                                                         std::bind(&AdaptiveCruiseControl::get_ir_data, this, std::placeholders::_1));
}

int AdaptiveCruiseControl::get_ir_data(ir_msg::SharedPtr ir_data) {
    auto reading = ir_data->readings;

    for (size_t i = 0; i < reading.size(); i++)
        RCLCPP_INFO(this->get_logger(), "ir: %zu: %i",i, reading[i].value);

    return 0;
}

void AdaptiveCruiseControl::get_one_side_speed(RelativeSpeedValue& speed_value, int left_side, int right_side, const lidar_scan::SharedPtr scan_data) {
    auto time_between_scans = scan_data->scan_time;
    auto time_between_measurements = scan_data->time_increment;
    auto min_range = scan_data->range_min;
    auto ranges = scan_data->ranges;

    double cum_dist = 0;
    size_t count_dist = 0;

    auto r = right_side*ranges.size()/36;
    auto l = left_side*ranges.size()/36;

    auto inc_time = static_cast<double>(l-r) * time_between_measurements;

//    RCLCPP_INFO(this->get_logger(), "time %i %i: %f m", right_side, left_side, inc_time);
//    RCLCPP_INFO(this->get_logger(), "time 2 %i %i: %f m", right_side, left_side, time_between_measurements);
//



    for (size_t i = r; i < l; i++) {
        if (ranges[i] >= min_range && ranges[i] <= 5) {
            cum_dist += ranges[i];
            count_dist++;
        }
    }

    double distance = count_dist ? (cum_dist / static_cast<double>(count_dist)) : std::numeric_limits<double>::max();
    speed_value.distance_dbg = distance;

    RCLCPP_INFO(this->get_logger(), "distance %i %i: %f m", right_side, left_side, distance);

    if (speed_value.prev_dist < 0.000001 && count_dist != 0) {

        speed_value.prev_dist = distance;
    } else if (count_dist != 0) {

        speed_value.curr_dist = distance;
        speed_value.rel_speed = (speed_value.curr_dist-speed_value.prev_dist ) / time_between_scans;
        speed_value.prev_dist = speed_value.curr_dist;
    } else {

        speed_value.curr_dist = 0;
        speed_value.prev_dist = 0;
    }

    RCLCPP_INFO(this->get_logger(), "relative speed %i %i: %f m/s", right_side, left_side, speed_value.rel_speed);
}


int AdaptiveCruiseControl::get_lidar_data(const lidar_scan::SharedPtr scan_data) {
    auto cmd_msg = twist_msg();

    get_one_side_speed(left_side_view, 11, 10, scan_data);
    get_one_side_speed(right_side_view, 8, 7, scan_data);
    get_one_side_speed(center_view, 10, 8, scan_data);

    own_speed = odom_speed; // current speed based on imu

    // change based on center view
    if (center_view.distance_dbg >= safe_distance || (center_view.rel_speed + own_speed - desired_speed) > 0.001) {
        own_speed = desired_speed;
    } else if (center_view.distance_dbg <= critical_distance || center_view.rel_speed+own_speed < 0) {
        own_speed = 0;
        // this means the car is going on us ^)
    } else {
        own_speed += center_view.rel_speed;
    }

    // if there is something on either side - reduce speed by 10%

    RCLCPP_INFO(this->get_logger(), "left: %f m/s", left_side_view.curr_dist);
    RCLCPP_INFO(this->get_logger(), "right: %f m/s", right_side_view.curr_dist);

    if (left_side_view.curr_dist < safe_distance/2 || left_side_view.curr_dist < safe_distance/2 ) {
        own_speed *= 0.9;
    }

    if (left_side_view.curr_dist < critical_distance || left_side_view.curr_dist < critical_distance) {
        own_speed = 0;
    }

    cmd_msg.linear.x = own_speed;
//    cmd_msg.angular.z = own_speed;
    cmd_publisher_->publish(cmd_msg);

    RCLCPP_INFO(this->get_logger(), "Speed: %f m/s", own_speed);

    return 0;
}

int AdaptiveCruiseControl::get_odom_data([[maybe_unused]] odom_msg::SharedPtr odom_data) {
//    RCLCPP_INFO(this->get_logger(), "Estimated vel %f", odom_data->twist.twist.linear.x);
    odom_speed = odom_data->twist.twist.linear.x;
    return 0;

}

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveCruiseControl>());
    rclcpp::shutdown();
    return 0;
}
