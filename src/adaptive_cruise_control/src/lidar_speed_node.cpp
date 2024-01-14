#include "adaptive_cruise_control/lidar_speed_node.h"

#include <chrono>

#include <sys/socket.h>
#include <netinet/ip.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>

using namespace std::chrono_literals;

LidarSpeed::LidarSpeed() : Node("adaptive_cruise_control"),
                                                 desired_speed{0.4},
//                                                 prev_dist{0},
//                                                 curr_dist{0},
                                                 own_speed{desired_speed},
                                                 safe_distance{2},
                                                 critical_distance{0.4},
                                                 odom_speed{0},
                                                 speed_camera_lin{0},
                                                 speed_camera_ang{0},
                                                 new_data{false},
                                                 sfd{0}
{
    lidar_subscriber_ = this->create_subscription<lidar_scan>("/scan",
                                                                               rclcpp::SensorDataQoS(),
                                                                               std::bind(&LidarSpeed::get_lidar_data, this, std::placeholders::_1));

    odom_subscriber_ = this->create_subscription<odom_msg>("/odom",
                                                              rclcpp::SensorDataQoS(),
                                                              std::bind(&LidarSpeed::get_odom_data, this, std::placeholders::_1));

    cmd_publisher_ = this->create_publisher<twist_msg>("/cmd_vel",
                                                              rclcpp::SensorDataQoS());

    client_ = this->create_client<speed_srv>("get_speed");


//    // open socket

    sfd = socket(AF_INET, SOCK_STREAM, 0);
//
//    if (sfd < 0) {
//        RCLCPP_ERROR(this->get_logger(), "socket error");
////        return CREATE_SOCKET_ERR;
//    }
//    // bind
//    if (bind(sfd, (struct sockaddr*) &server_info, sizeof (struct sockaddr_in)) < 0) {
//        RCLCPP_ERROR(this->get_logger(),"bind error");
////        return BIND_ERROR;
//    }
//    // listen
//
//    if (listen(sfd, 0) < 0) {
//        RCLCPP_ERROR(this->get_logger(),"listen error");
////        return LISTEN_ERROR;
//    }

}


void LidarSpeed::get_one_side_speed(RelativeSpeedValue& speed_value, int left_side, int right_side, const lidar_scan::SharedPtr scan_data) {
    auto time_between_scans = scan_data->scan_time;
    auto time_between_measurements = scan_data->time_increment;
    auto min_range = scan_data->range_min;
    auto ranges = scan_data->ranges;

    double cum_dist = 0;
    size_t count_dist = 0;

    auto r = right_side*ranges.size()/36;
    auto l = left_side*ranges.size()/36;

    auto inc_time = static_cast<double>(l-r) * time_between_measurements;


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


int LidarSpeed::get_lidar_data(const lidar_scan::SharedPtr scan_data) {
    auto request = std::make_shared<speed_srv::Request>();
    // Wait for the result.


//    while (!client_->wait_for_service(1s)) {
//        if (!rclcpp::ok()) {
//            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//        }
//        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//    }
//
//    auto result = client_->async_send_request(request, std::bind(&LidarSpeed::get_speed_callback, this, std::placeholders::_1));



    auto cmd_msg = twist_msg();

    get_one_side_speed(left_side_view, 11, 10, scan_data);
    get_one_side_speed(right_side_view, 8, 7, scan_data);
    get_one_side_speed(center_view, 10, 8, scan_data);

    auto own_speed_loc = odom_speed; // current speed based on imu

    // change based on center view
    if (center_view.distance_dbg >= safe_distance || (center_view.rel_speed + own_speed - desired_speed) > 0.001) {
        own_speed_loc = desired_speed;
    } else if (center_view.distance_dbg <= critical_distance || center_view.rel_speed+own_speed < 0) {
        own_speed_loc = 0;
        // this means the car is going on us ^)
    } else {
        own_speed_loc += center_view.rel_speed;
    }

    // if there is something on either side - reduce speed by 10%

//    RCLCPP_INFO(this->get_logger(), "left: %f m/s", left_side_view.curr_dist);
//    RCLCPP_INFO(this->get_logger(), "right: %f m/s", right_side_view.curr_dist);

//    if (left_side_view.curr_dist < safe_distance/2 || left_side_view.curr_dist < safe_distance/2 ) {
//        own_speed_loc *= 0.9;
//    }
//
//    if (left_side_view.curr_dist < critical_distance || left_side_view.curr_dist < critical_distance) {
//        own_speed_loc = 0;
//    }

    if (new_data) {
        cmd_msg.linear.x = std::min(speed_camera_lin, own_speed_loc);
        cmd_msg.angular.z = speed_camera_ang;
        new_data = false;
        RCLCPP_INFO(this->get_logger(), "got data");


    } else {
        RCLCPP_INFO(this->get_logger(), "not got");

        cmd_msg.linear.x = own_speed_loc;
    }

//        cmd_publisher_->publish(cmd_msg);

    RCLCPP_INFO(this->get_logger(), "Speed: %f m/s", own_speed_loc);

    own_speed = own_speed_loc;

    return 0;
}

int LidarSpeed::get_odom_data([[maybe_unused]] odom_msg::SharedPtr odom_data) {


    odom_speed = odom_data->twist.twist.linear.x;

    int status{};

    struct sockaddr_in server_info = {0};
    server_info.sin_family = AF_INET;
    server_info.sin_port = htons(1338);
    inet_pton(AF_INET, "192.168.185.5", &server_info.sin_addr.s_addr);


    if ((status = connect(sfd, (struct sockaddr*)&server_info,
                           sizeof(server_info))) < 0) {
        RCLCPP_ERROR(this->get_logger(),"\nConnection Failed \n");
//        return -1;
    }

    std::string to_send = std::to_string(odom_speed);

    send(sfd, to_send.c_str(), to_send.length(), 0);


    // closing the connected socket
    close(sfd);


    return 0;

}

void LidarSpeed::get_speed_callback(rclcpp::Client<speed_srv>::SharedFuture future) {
    speed_camera_ang = future.get()->angular_z;
    speed_camera_lin = future.get()->linear_x;

    new_data = true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSpeed>());
    rclcpp::shutdown();
    return 0;
}
