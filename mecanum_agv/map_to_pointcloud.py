#!/usr/bin/env python3
"""Republishes /map (nav_msgs/OccupancyGrid) as a colored /map_points PointCloud2.

Workaround for a long-standing, unresolved upstream RViz2 bug where the Map
display's palette shader fails to link ("active samplers with a different
type refer to the same texture image unit", ros2/rviz#1279) on a wide range
of GPUs. PointCloud2 uses an unrelated, working shader path (the same one
LaserScan already renders fine with), so this lets the map be viewed live in
RViz without touching RViz/Ogre itself.
"""
import struct

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField

FREE_RGB = (170, 170, 170)
OCCUPIED_RGB = (255, 40, 40)


def _rgb_float(r, g, b):
    packed = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
    return packed


class MapToPointCloud(Node):

    def __init__(self):
        super().__init__('map_to_pointcloud')
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._callback, map_qos)
        self.pub = self.create_publisher(PointCloud2, '/map_points', map_qos)
        self.get_logger().info('Republishing /map as /map_points (PointCloud2)')

    def _callback(self, msg: OccupancyGrid):
        width, height = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape((height, width))
        rows, cols = np.where(data >= 0)
        if rows.size == 0:
            return

        xs = ox + (cols.astype(np.float32) + 0.5) * res
        ys = oy + (rows.astype(np.float32) + 0.5) * res
        zs = np.zeros_like(xs)

        occupied = data[rows, cols] >= 50
        free_rgb = _rgb_float(*FREE_RGB)
        occ_rgb = _rgb_float(*OCCUPIED_RGB)
        rgbs = np.where(occupied, occ_rgb, free_rgb).astype(np.float32)

        points = np.column_stack((xs, ys, zs, rgbs)).astype(np.float32)

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = points.tobytes()

        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = MapToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
