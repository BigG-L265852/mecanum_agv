#!/usr/bin/env python3
"""Subscribes to /map and writes it to a PNG whenever an update arrives.

Workaround for the rviz Map display's indexed_8bit_image shader failing to link
on this machine's Mesa/Iris Xe driver. Run alongside slam.launch.py:

    ros2 run mecanum_agv map_to_png.py [output_path]
"""
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node


class MapToPng(Node):

    def __init__(self, output_path):
        super().__init__('map_to_png')
        self.output_path = output_path
        self.create_subscription(OccupancyGrid, '/map', self._callback, 10)
        self.get_logger().info(f'Writing /map to {output_path} on every update')

    def _callback(self, msg: OccupancyGrid):
        width, height = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((height, width))

        # OccupancyGrid: -1 unknown, 0 free, 100 occupied -> grayscale, unknown mid-gray.
        img = np.full((height, width), 128, dtype=np.uint8)
        img[data == 0] = 254
        img[data == 100] = 0
        occupied_or_free = (data >= 0)
        img[occupied_or_free] = 254 - (data[occupied_or_free] * 254 // 100).astype(np.uint8)

        # OccupancyGrid row 0 is the bottom of the map; flip for a normal top-down image.
        Image.fromarray(np.flipud(img), mode='L').save(self.output_path)
        self.get_logger().info(f'Saved {width}x{height} map to {self.output_path}')


def main():
    output_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/slam_map.png'
    rclpy.init()
    node = MapToPng(output_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
