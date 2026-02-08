import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class ImuOrientationPlotter(Node):
    def __init__(self):
        super().__init__("imu_orientation_plotter")

        self.declare_parameter("imu_topic", "bno085/imu")
        self.declare_parameter("history_secs", 60.0)
        self.declare_parameter("max_points", 6000)
        self.declare_parameter("use_header_stamp", True)

        self.imu_topic = self.get_parameter("imu_topic").value
        self.history_secs = float(self.get_parameter("history_secs").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.use_header_stamp = bool(self.get_parameter("use_header_stamp").value)

        qos = QoSProfile(depth=50)
        self.sub = self.create_subscription(Imu, self.imu_topic, self._imu_cb, qos)

        self._t0 = None
        self._lock = threading.Lock()
        self._t = deque()
        self._roll = deque()
        self._pitch = deque()
        self._yaw = deque()
        self._qnorm = deque()

    def _imu_cb(self, msg: Imu):
        if self.use_header_stamp and msg.header.stamp.sec != 0:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            now = self.get_clock().now().to_msg()
            t = now.sec + now.nanosec * 1e-9

        if self._t0 is None:
            self._t0 = t
        t = t - self._t0

        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)

        qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if qnorm > 0.0:
            qx /= qnorm
            qy /= qnorm
            qz /= qnorm
            qw /= qnorm

        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        with self._lock:
            self._t.append(t)
            self._roll.append(math.degrees(roll))
            self._pitch.append(math.degrees(pitch))
            self._yaw.append(math.degrees(yaw))
            self._qnorm.append(qnorm)

            self._trim_history()

    def _trim_history(self):
        while len(self._t) > self.max_points:
            self._t.popleft()
            self._roll.popleft()
            self._pitch.popleft()
            self._yaw.popleft()
            self._qnorm.popleft()

        if self.history_secs > 0 and len(self._t) > 1:
            newest = self._t[-1]
            while len(self._t) > 1 and (newest - self._t[0]) > self.history_secs:
                self._t.popleft()
                self._roll.popleft()
                self._pitch.popleft()
                self._yaw.popleft()
                self._qnorm.popleft()

    def snapshot(self):
        with self._lock:
            return (
                list(self._t),
                list(self._roll),
                list(self._pitch),
                list(self._yaw),
                list(self._qnorm),
            )


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main():
    rclpy.init()
    node = ImuOrientationPlotter()

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    fig.suptitle("IMU Orientation Over Time")

    (line_roll,) = ax1.plot([], [], label="roll (deg)")
    (line_pitch,) = ax1.plot([], [], label="pitch (deg)")
    (line_yaw,) = ax1.plot([], [], label="yaw (deg)")
    ax1.set_ylabel("deg")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    (line_qnorm,) = ax2.plot([], [], label="|q|")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("quat norm")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")

    def update(_):
        t, roll, pitch, yaw, qnorm = node.snapshot()
        if not t:
            return line_roll, line_pitch, line_yaw, line_qnorm

        line_roll.set_data(t, roll)
        line_pitch.set_data(t, pitch)
        line_yaw.set_data(t, yaw)
        line_qnorm.set_data(t, qnorm)

        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        return line_roll, line_pitch, line_yaw, line_qnorm

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    anim = FuncAnimation(fig, update, interval=100, blit=False)
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if anim:
            pass


if __name__ == "__main__":
    main()
