#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from time import sleep

from adaptive_cruise_control.lane_detection import getLaneCurve


LEN_AVG = 10

class RoadDetectionNode(Node):

    def __init__(self):
        super().__init__('road_detection')

        self.image_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", qos_profile_sensor_data)
        self.velocities = []
        self.weights = np.linspace(0, 1, LEN_AVG)

    def image_callback(self, image_msg):
        width, height = image_msg.width, image_msg.height
        frame = np.array(image_msg.data).reshape(height, width, 3)
        curveVal1 = -getLaneCurve(frame, 2)
        curveVal = curveVal1

        sensitivity = 1.3  # MAX SPEED
        maxVAl = 70
        curveVal = min(curveVal, maxVAl)
        curveVal = max(curveVal, -maxVAl)
        thres = 5
        if 0 < curveVal < thres:
            curveVal = 0
        elif 0 > curveVal > -thres:
            curveVal = 0

        if np.abs(curveVal) < 10:
            self.velocities.append(0.4)
        if np.abs(curveVal) < 15:
            self.velocities.append(0.25)
        else:
            self.velocities.append(0.1)

        if len(self.velocities) > 15:
            self.velocities.pop(0)
            linear_velocity = np.average(self.velocities, weights=self.weights, axis=-1)
        else:
            linear_velocity = np.average(self.velocities)
        # linear_velocity = 0.1
        self.get_logger().info(f'linear_velocity: {linear_velocity}, curveVal: {curveVal}')


        msg_to_send = Twist()
        curveVal = np.deg2rad(-curveVal)

        msg_to_send.linear.x = linear_velocity
        msg_to_send.angular.z = curveVal

        cv2.waitKey(1)
        self.cmd_vel_publisher.publish(msg_to_send)
        sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
