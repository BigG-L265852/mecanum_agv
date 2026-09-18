#!/usr/bin/env python3
"""Live-updating map view, outside RViz (workaround for the RViz Map-display shader bug).

Subscribes to /map and (optionally) the map->base_frame TF, and redraws an OpenCV
window on every update with the robot's current position/heading marked on it.
Works over the network like any ROS 2 topic/TF — no changes needed to watch a
robot on the Raspberry Pi from here, as long as both sides share a ROS_DOMAIN_ID
and can reach each other (same LAN, or a discovery/RMW setup that routes between them).

    ros2 run mecanum_agv map_viewer
"""
import cv2
import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

SCALE = 4  # pixels per map cell, for visibility


class MapViewer(Node):

    def __init__(self):
        super().__init__('map_viewer')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('window_name', 'SLAM map (live)')

        self.map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.window_name = self.get_parameter('window_name').value

        self.latest_map = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, 10)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.create_timer(0.2, self._redraw)  # 5 Hz is plenty for a map view
        self.get_logger().info(f'Watching {self.map_topic}, close the window or Ctrl+C to stop')

    def _map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _robot_pixel_pose(self, msg: OccupancyGrid):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                msg.header.frame_id, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        wx = tf.transform.translation.x
        wy = tf.transform.translation.y

        q = tf.transform.rotation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        px = (wx - ox) / res
        py = msg.info.height - (wy - oy) / res  # flip to match flipud'd image
        return px, py, yaw

    def _redraw(self):
        if self.latest_map is None:
            return
        msg = self.latest_map
        width, height = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((height, width))

        img = np.full((height, width), 128, dtype=np.uint8)
        occupied_or_free = (data >= 0)
        img[occupied_or_free] = 254 - (data[occupied_or_free] * 254 // 100).astype(np.uint8)
        img = np.flipud(img)

        color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        color = cv2.resize(color, (width * SCALE, height * SCALE), interpolation=cv2.INTER_NEAREST)

        pose = self._robot_pixel_pose(msg)
        if pose is not None:
            px, py, yaw = pose
            cx, cy = int(px * SCALE), int(py * SCALE)
            cv2.circle(color, (cx, cy), max(4, SCALE), (0, 0, 255), -1)
            hx = int(cx + 3 * SCALE * np.cos(yaw))
            hy = int(cy - 3 * SCALE * np.sin(yaw))
            cv2.line(color, (cx, cy), (hx, hy), (0, 0, 255), 2)

        cv2.imshow(self.window_name, color)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = MapViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
