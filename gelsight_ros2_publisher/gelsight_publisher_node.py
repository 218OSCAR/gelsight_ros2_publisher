#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 node for streaming GelSight Mini images as sensor_msgs/Image.

Reads configuration from gs_sdk examples/configs/gsmini.yaml
and publishes /gelsight/image_raw at the configured frame rate.
"""

import os
import sys
import cv2
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
sys.path.append(os.path.expanduser("~/gs_sdk"))
from gs_sdk.gs_device import FastCamera



class GelSightPublisher(Node):
    def __init__(self):
        super().__init__('gelsight_publisher')

        # parameters
        self.declare_parameter('config_path', os.path.expanduser('~/gs_sdk/examples/configs/gsmini.yaml'))
        self.declare_parameter('topic_name', '/gelsight/image_raw')

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        self.device_name = cfg["device_name"]
        self.imgh = cfg["imgh"]
        self.imgw = cfg["imgw"]
        self.raw_imgh = cfg["raw_imgh"]
        self.raw_imgw = cfg["raw_imgw"]
        self.framerate = cfg["framerate"]

        # iniitialize device
        self.device = FastCamera(
            self.device_name,
            self.imgh,
            self.imgw,
            self.raw_imgh,
            self.raw_imgw,
            self.framerate
        )
        self.device.connect()

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic_name, 10)
        self.timer = self.create_timer(1.0 / self.framerate, self.timer_callback)
        self.get_logger().info(f"Connected to {self.device_name}, publishing at {self.framerate} Hz")

    def timer_callback(self):
        try:
            frame = self.device.get_image()
            if frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "gelsight_frame"
                self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error getting image: {e}")

    def destroy_node(self):
        self.device.release()
        self.get_logger().info("Device released.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GelSightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
