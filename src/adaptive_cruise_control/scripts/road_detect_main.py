#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image

from adaptive_cruise_control.lane import Lane

class RoadDetectionNode(Node):
    def __init__(self):
        super().__init__('road_detection')

        self.image_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

    def image_callback(self, image_msg):
        # self.get_logger().info(f"data: {image_msg.data}")

        frame = image_msg.data
        self.get_logger().info(f"data type: {type(image_msg.data)}")

        lane_obj = Lane(orig_frame=frame)
        lane_obj.get_line_markings()
        lane_obj.plot_roi(plot=False)

        lane_obj.perspective_transform(plot=False)
        lane_obj.calculate_histogram(plot=False)
        left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(
            plot=False)

        lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)

        curvature = lane_obj.calculate_curvature(print_to_terminal=False)
        curvature = sum(curvature) / 2

        self.get_logger().info(f"curvature: {curvature}")


def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
