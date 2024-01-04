#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
import numpy as np
import cv2

from adaptive_cruise_control.lane import Lane


class RoadDetectionNode(Node):

    def __init__(self):
        super().__init__('road_detection')

        self.image_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        self.i = 0

    def image_callback(self, image_msg):
        width, height = image_msg.width, image_msg.height
        frame = np.array(image_msg.data).reshape(height, width, 3)
        # import matplotlib.pyplot as plt
        # plt.imshow(frame)
        # plt.show()
        # exit()
        # self.get_logger().info(f"{width}x{height}. frame: {frame.shape}")

        # lane_obj = Lane(orig_frame=frame)
        # lane_obj.get_line_markings()
        # lane_obj.plot_roi(plot=False)
        #
        # lane_obj.perspective_transform(plot=False)
        # lane_obj.calculate_histogram(plot=False)
        # left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(
        #     plot=False)
        #
        # lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
        #
        # curvature = lane_obj.calculate_curvature(print_to_terminal=False)
        # curvature = (curvature[0] + curvature[1]) / 2

        # self.get_logger().info(f"curvature: {curvature}")
        self.i += 1
        if self.i % 5 == 0:
            cv2.imwrite(f"outputs_2/{self.i}_frame.png", frame)
            self.get_logger().info(f"i: {self.i}")

        # frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
        # frame_with_lane_lines2 = lane_obj.display_curvature_offset(
        #     frame=frame_with_lane_lines, plot=True)




def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
