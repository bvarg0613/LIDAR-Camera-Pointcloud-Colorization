#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import StaticTransformBroadcaster


def quat_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    # roll (X), pitch (Y), yaw (Z), radians
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def make_static_tf(parent: str, child: str,
                   x: float, y: float, z: float,
                   roll: float, pitch: float, yaw: float) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child

    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)

    t.transform.rotation = quat_from_euler(roll, pitch, yaw)
    return t


class StaticRigTF(Node):
    def __init__(self):
        super().__init__("static_rig_tf")
        self.broadcaster = StaticTransformBroadcaster(self)

        base = "base_link"

        # IMPORTANT: use os_sensor so your transforms connect to the Ouster driver's frames
        # (os_sensor -> os_lidar and os_sensor -> os_imu already exist on /tf_static).
        transforms = [
            make_static_tf(base, "os_sensor", 0.05085, 0, 0.1618, 0.615, -0.524, 2.186),

            make_static_tf(base, "camera",  -0.02891, 0, 0.17417, -0.785, 0.0, 1.5707963267948966),
            make_static_tf(base, "camera2",  0.01613, -0.06965, 0.12855, 1.5707963267948966, 3.926, 0.0),
            make_static_tf(base, "camera3",  0.06231, 0, 0.08296, 3.927, 0.0, 4.712),
            make_static_tf(base, "camera4",  0.01613, 0.06965, 0.12855, 1.5707963267948966, -3.926, -3.141592653589793),

        ]

        stamp = self.get_clock().now().to_msg()
        for t in transforms:
            t.header.stamp = stamp

        self.broadcaster.sendTransform(transforms)
        self.get_logger().info("Published static TFs: base_link -> os_sensor and base_link -> cam1..cam4")


def main():
    rclpy.init()
    node = StaticRigTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
