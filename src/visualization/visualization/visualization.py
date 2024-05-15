import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import cv2
import numpy as np
import math

class Visualization(Node):

    def __init__(self):
        super().__init__('visualization')
        self.size_ = 1000
        self.scale_ = 100

        self.subscription = self.create_subscription(
            LaserScan,
            '/hokuyo',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        display = np.zeros((self.size_, self.size_, 3), dtype=np.uint8)

        for i in range(len(msg.ranges)):
            if(msg.ranges[i] >= msg.range_max or msg.ranges[i] <= msg.range_min):
                continue
            angle = (i * msg.angle_increment) - math.radians(30)

            x = int(msg.ranges[i] * np.cos(angle) * self.scale_)
            y = int(msg.ranges[i] * np.sin(angle) * self.scale_)
            x_display = int(self.size_ / 2 + x)
            y_display = int(self.size_ / 2 - y)

            cv2.circle(display, (int(x_display), int(y_display)), 1, (0, 0, 255), -1)
            print(x_display, y_display)

        cv2.circle(display, (int(self.size_/2), int(self.size_/2)), 1, (255, 0, 0), -1)

        cv2.imshow('Display', display)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = Visualization()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

