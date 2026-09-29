#!/usr/bin/env python3
"""Republishes /map (nav_msgs/OccupancyGrid) as a colored /map_points PointCloud2.

Workaround for a long-standing, unresolved upstream RViz2 bug where the Map
display's palette shader fails to link ("active samplers with a different
type refer to the same texture image unit", ros2/rviz#1279) on a wide range
of GPUs. PointCloud2 uses an unrelated, working shader path (the same one
LaserScan already renders fine with), so this lets the map be viewed live in
RViz without touching RViz/Ogre itself.

Parameters let one more instance per Nav2 costmap do the same for
/global_costmap/costmap and /local_costmap/costmap (mode:=costmap): only
cells with cost > 0 are published, colored by cost, slightly above the map.
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
LETHAL_RGB = (255, 0, 255)      # cost 100: obstacle cell
INSCRIBED_RGB = (0, 200, 255)   # cost 99: footprint centre here = collision


def _rgb_float(r, g, b):
    packed = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
    return packed


class MapToPointCloud(Node):

    def __init__(self):
        super().__init__('map_to_pointcloud')
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_topic', '/map_points')
        self.declare_parameter('mode', 'map')  # 'map' or 'costmap'
        self.declare_parameter('z_offset', 0.0)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.mode = self.get_parameter('mode').value
        self.z_offset = float(self.get_parameter('z_offset').value)

        if self.mode == 'costmap':
            # Costmaps are republished continuously, so a volatile subscription is
            # enough (and is QoS-compatible with any publisher durability).
            qos = QoSProfile(depth=1)
        else:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, input_topic, self._callback, qos)
        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.get_logger().info(
            f'Republishing {input_topic} as {output_topic} (PointCloud2, mode={self.mode})')

    def _callback(self, msg: OccupancyGrid):
        width, height = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape((height, width))
        if self.mode == 'costmap':
            rows, cols = np.where(data > 0)
        else:
            rows, cols = np.where(data >= 0)
        if rows.size == 0:
            return

        xs = ox + (cols.astype(np.float32) + 0.5) * res
        ys = oy + (rows.astype(np.float32) + 0.5) * res
        zs = np.full_like(xs, self.z_offset)

        values = data[rows, cols]
        if self.mode == 'costmap':
            rgbs = self._costmap_colors(values)
        else:
            occupied = values >= 50
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

    @staticmethod
    def _costmap_colors(values):
        """Inflated cost 1..98 fades yellow -> red; inscribed and lethal get their own color."""
        t = (values.astype(np.float32) - 1.0) / 97.0
        r = np.full(values.shape, 255, dtype=np.uint32)
        g = (220 * (1.0 - np.clip(t, 0.0, 1.0))).astype(np.uint32)
        b = np.zeros(values.shape, dtype=np.uint32)
        packed = (r << 16) | (g << 8) | b
        packed[values == 99] = (INSCRIBED_RGB[0] << 16) | (INSCRIBED_RGB[1] << 8) | INSCRIBED_RGB[2]
        packed[values >= 100] = (LETHAL_RGB[0] << 16) | (LETHAL_RGB[1] << 8) | LETHAL_RGB[2]
        return packed.view(np.float32)


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
