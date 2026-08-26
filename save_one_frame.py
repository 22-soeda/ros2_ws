#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class OneShotSaver(Node):
    def __init__(self, topic, outfile):
        super().__init__('one_shot_image_saver')
        self.bridge = CvBridge()
        self.outfile = outfile
        self.saved = False
        self.sub = self.create_subscription(Image, topic, self.cb, 10)

    def cb(self, msg):
        if self.saved:
            return
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(self.outfile, cv_img)
        self.get_logger().info(f'Saved {msg.width}x{msg.height} ({msg.encoding}) to {self.outfile}')
        self.saved = True

def main():
    topic = sys.argv[1]
    outfile = sys.argv[2]
    rclpy.init()
    node = OneShotSaver(topic, outfile)
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
