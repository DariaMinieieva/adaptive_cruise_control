#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from service_speed_interfaces.srv import Velocities

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from time import sleep

from adaptive_cruise_control.lane_detection import getLaneCurve


LEN_AVG = 30

from rclpy.callback_groups import ReentrantCallbackGroup

from threading import Event

class RoadDetectionNode(Node):

    def __init__(self):
        super().__init__('road_detection')
        self.callback_group = ReentrantCallbackGroup()

        self.image_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", qos_profile_sensor_data)
        self.velocities = []
        self.weights = np.linspace(0, 1, LEN_AVG - 5)
        self.weights = np.append(self.weights, self.weights[-5:])
        self.bins = np.array([0, 5, 30, 50, 90])
        self.corresponding_vels = np.array([0.4, 0.25, 0.1, 0.05])


        self.srv = self.create_service(Velocities, 'get_speed', self.speed_callback)


        self.lin_to_send = 0
        self.ang_to_send = 0



    def image_callback(self, image_msg):
        width, height = image_msg.width, image_msg.height
        frame = np.array(image_msg.data).reshape(height, width, 3)
        curveVal1 = -getLaneCurve(frame, 2)
        curveVal = curveVal1

        maxVAl = 90
        curveVal = min(curveVal, maxVAl)
        curveVal = max(curveVal, -maxVAl)
        thres = 5
        if 0 < curveVal < thres:
            curveVal = 0
        elif 0 > curveVal > -thres:
            curveVal = 0

        # if np.abs(curveVal) < 10:
        #     self.velocities.append(0.4)
        # if np.abs(curveVal) < 15:
        #     self.velocities.append(0.25)
        # else:
        #     self.velocities.append(0.1)
        vel = self.corresponding_vels[np.digitize(abs(curveVal), self.bins) - 1]
        self.velocities.append(vel)

        if len(self.velocities) > LEN_AVG:
            self.velocities.pop(0)
            linear_velocity = np.average(self.velocities, weights=self.weights, axis=-1)
        else:
            linear_velocity = np.average(self.velocities)
        # linear_velocity = 0.1
        self.get_logger().info(f'linear_velocity: {round(vel, 2)}, {round(linear_velocity, 2)}, curveVal: {round(curveVal, 2)}')

        msg_to_send = Twist()
        curveVal = np.deg2rad(-curveVal)

        msg_to_send.linear.x = linear_velocity
        self.lin_to_send = linear_velocity
        self.ang_to_send = curveVal
        msg_to_send.angular.z = curveVal
        # self.cmd_vel_publisher.publish(msg_to_send)

        cv2.waitKey(1)
        sleep(0.1)

    def speed_callback(self, request, response):
        response.linear_x = float(self.lin_to_send) # set speed here
        response.angular_z =  float(self.ang_to_send) # set speed here

        return response





def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
