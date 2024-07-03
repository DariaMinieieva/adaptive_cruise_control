//
// Created by turtlehouse on 13.01.24.
//

#include "adaptive_cruise_control/main_cruise_control.h"
#include <chrono>

using namespace std::chrono_literals;

ACC::ACC(): Node("ACC") {
    client_ =
            this->create_client<speed_srv>("get_speed_from_lidar");

    timer_ = this->create_wall_timer(
            500ms, std::bind(&ACC::combine_velocities, this));

}

void ACC::combine_velocities() {
    auto request = std::make_shared<speed_srv::Request>();
    // Wait for the result.


    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ok");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service kill");
    }

}

void ACC::response_callback(rclcpp::Client<speed_srv>::SharedFuture future) {
    RCLCPP_INFO(this->get_logger(), "response callback");
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "Result: success: speed %f", future.get()->linear_x);
    } else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
    RCLCPP_INFO(this->get_logger(), "end response callback");

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ACC>());
    rclcpp::shutdown();
    return 0;
}