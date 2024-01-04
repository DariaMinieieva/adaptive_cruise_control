#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image

class RoadDetectionNode(Node):
    def __init__(self):
        super().__init__('road_detection')

        self.image_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

    def image_callback(self, image_msg):
        self.get_logger().info(f"data: {image_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
