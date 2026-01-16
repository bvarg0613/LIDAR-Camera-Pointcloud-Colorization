#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import message_filters

import tf2_ros
import tf2_geometry_msgs

import struct


def pack_rgb(r: int, g: int, b: int) -> float:
    rgb_uint32 = (r << 16) | (g << 8) | b
    return struct.unpack("f", struct.pack("I", rgb_uint32))[0]


class PointCloudColorizer(Node):
    def __init__(self):
        super().__init__("pointcloud_colorizer")
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_models = {}   # frame -> PinholeCameraModel
        self.camera_images = {}   # frame -> cv image (BGR)

        for cam_id in range(1, 5):
            img_sub = message_filters.Subscriber(self, Image, f"/camera{cam_id}/image_raw", qos_profile=qos_profile_sensor_data)
            info_sub = message_filters.Subscriber(self, CameraInfo, f"/camera{cam_id}/camera_info", qos_profile=qos_profile_sensor_data)
            ts = message_filters.ApproximateTimeSynchronizer([img_sub, info_sub], 10, 0.1)
            ts.registerCallback(self.get_camera_callback(cam_id))

        self.create_subscription(PointCloud2, "/ouster/points", self.pointcloud_callback, qos_profile_sensor_data)
        self.colorized_pub = self.create_publisher(PointCloud2, "/colorized_points", qos_profile_sensor_data)

    def get_camera_callback(self, cam_id: int):
        def callback(img_msg: Image, info_msg: CameraInfo):
            try:
                cam_model = PinholeCameraModel()
                cam_model.fromCameraInfo(info_msg)

                cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

                # cam1 TF frame is "camera" (not "camera1") per your static TF
                cam_frame = "camera" if cam_id == 1 else f"camera{cam_id}"

                self.camera_models[cam_frame] = cam_model
                self.camera_images[cam_frame] = cv_img
            except Exception as e:
                self.get_logger().warn(f"Camera {cam_id} callback error: {e}")

        return callback

    def pointcloud_callback(self, cloud_msg: PointCloud2):
        if not self.camera_models:
            return

        colored_points = []

        # Convert ROS msg stamp -> rclpy Time (fixes Pylance + correct API usage)
        cloud_time = Time.from_msg(cloud_msg.header.stamp)

        for x, y, z in point_cloud2.read_points(
            cloud_msg, field_names=["x", "y", "z"], skip_nans=True
        ):
            best_rgb = None

            for cam_frame, cam_model in self.camera_models.items():
                img = self.camera_images.get(cam_frame)
                if img is None:
                    continue

                try:
                    ps = PointStamped()
                    ps.header = cloud_msg.header
                    ps.point.x = float(x)
                    ps.point.y = float(y)
                    ps.point.z = float(z)

                    tf = self.tf_buffer.lookup_transform(
                        cam_frame,
                        cloud_msg.header.frame_id,
                        cloud_time,
                    )

                    p_cam = tf2_geometry_msgs.do_transform_point(ps, tf)

                    if p_cam.point.z <= 0.0:
                        continue

                    u, v = cam_model.project3dToPixel(
                        (p_cam.point.x, p_cam.point.y, p_cam.point.z)
                    )
                    u, v = int(u), int(v)

                    if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                        b, g, r = img[v, u]  # OpenCV BGR
                        best_rgb = (int(r), int(g), int(b))
                        break
                except Exception:
                    continue

            if best_rgb is not None:
                r, g, b = best_rgb
                rgb_f32 = pack_rgb(r, g, b)
                colored_points.append((float(x), float(y), float(z), rgb_f32))

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        out = point_cloud2.create_cloud(cloud_msg.header, fields, colored_points)
        self.colorized_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudColorizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
