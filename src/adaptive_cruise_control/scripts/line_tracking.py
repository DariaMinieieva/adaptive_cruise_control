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


        curveVal = getLaneCurve(img, 1)

        sen = 1.3  # SENSITIVITY
        maxVAl = 0.3  # MAX SPEED
        if curveVal > maxVAl:
            curveVal = maxVAl
        if curveVal < -maxVAl:
            curveVal = -maxVAl
        # print(curveVal)
        if curveVal > 0:
            sen = 1.7
            if curveVal < 0.05:
                curveVal = 0
        elif curveVal > -0.08:
            curveVal = 0
       # move


# cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
