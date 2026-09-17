"""cmd_vel -> mecanum wheel velocities over serial, encoder feedback -> /odom and TF."""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


class MecanumDriveNode(Node):

    def __init__(self):
        super().__init__('mecanum_drive_node')

        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation_x', 0.10)  # half of front-back wheelbase (lx)
        self.declare_parameter('wheel_separation_y', 0.10)  # half of left-right track width (ly)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_rate_hz', 30.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.r = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('wheel_separation_x').value
        self.ly = self.get_parameter('wheel_separation_y').value
        self.l = self.lx + self.ly
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        self._wheel_lock = threading.Lock()
        self._measured = (0.0, 0.0, 0.0, 0.0)  # w_fl, w_fr, w_rl, w_rr (rad/s)

        self.serial_conn = None
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        if serial is None:
            self.get_logger().error('pyserial not installed (apt install python3-serial)')
        else:
            try:
                self.serial_conn = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f'Connected to Arduino on {port} @ {baud}')
                reader = threading.Thread(target=self._serial_reader_loop, daemon=True)
                reader.start()
            except serial.SerialException as e:
                self.get_logger().warn(
                    f'Could not open {port} ({e}) — running without hardware, /odom will stay at origin')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)

        odom_rate = self.get_parameter('odom_rate_hz').value
        self.create_timer(1.0 / odom_rate, self._odom_timer_callback)
        self.create_timer(0.1, self._cmd_vel_watchdog_callback)

    def _inverse_kinematics(self, vx, vy, wz):
        """Body twist -> wheel angular velocities (rad/s). Assumes X-pattern rollers:
        front-left/rear-right rollers parallel, front-right/rear-left rollers parallel.
        Wiring this backwards makes the robot spin in place instead of strafing."""
        w_fl = (vx - vy - self.l * wz) / self.r
        w_fr = (vx + vy + self.l * wz) / self.r
        w_rl = (vx + vy - self.l * wz) / self.r
        w_rr = (vx - vy + self.l * wz) / self.r
        return w_fl, w_fr, w_rl, w_rr

    def _forward_kinematics(self, w_fl, w_fr, w_rl, w_rr):
        """Wheel angular velocities (rad/s) -> body twist (vx, vy, wz)."""
        vx = self.r / 4.0 * (w_fl + w_fr + w_rl + w_rr)
        vy = self.r / 4.0 * (-w_fl + w_fr + w_rl - w_rr)
        wz = self.r / (4.0 * self.l) * (-w_fl + w_fr - w_rl + w_rr)
        return vx, vy, wz

    def _cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        w_fl, w_fr, w_rl, w_rr = self._inverse_kinematics(msg.linear.x, msg.linear.y, msg.angular.z)
        self._send_wheel_targets(w_fl, w_fr, w_rl, w_rr)

    def _cmd_vel_watchdog_callback(self):
        """Stop the wheels if /cmd_vel goes silent, e.g. the controller node died or link dropped."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            self._send_wheel_targets(0.0, 0.0, 0.0, 0.0)

    def _send_wheel_targets(self, w_fl, w_fr, w_rl, w_rr):
        if self.serial_conn is None:
            return
        line = f'V {w_fl:.4f} {w_fr:.4f} {w_rl:.4f} {w_rr:.4f}\n'
        try:
            self.serial_conn.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write failed: {e}')

    def _serial_reader_loop(self):
        """Runs in a background thread: parses 'E w_fl w_fr w_rl w_rr' lines from the Arduino."""
        while rclpy.ok():
            try:
                raw = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException:
                time.sleep(0.1)
                continue
            if not raw.startswith('E '):
                continue
            parts = raw.split()
            if len(parts) != 5:
                continue
            try:
                w_fl, w_fr, w_rl, w_rr = (float(p) for p in parts[1:])
            except ValueError:
                continue
            with self._wheel_lock:
                self._measured = (w_fl, w_fr, w_rl, w_rr)

    def _odom_timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        with self._wheel_lock:
            w_fl, w_fr, w_rl, w_rr = self._measured
        vx, vy, wz = self._forward_kinematics(w_fl, w_fr, w_rl, w_rr)

        # Integrate in the robot frame, then rotate into odom before accumulating (holonomic: vy != 0).
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        quat_z = math.sin(self.theta / 2.0)
        quat_w = math.cos(self.theta / 2.0)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = quat_z
        t.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
