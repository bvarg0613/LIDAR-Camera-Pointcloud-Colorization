#!/usr/bin/env python3
import struct
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import message_filters

import tf2_ros

from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from sensor_msgs_py import point_cloud2


def pack_rgb(r: int, g: int, b: int) -> float:
    """Pack uint8 R,G,B into the PointCloud2 'rgb' float32 format RViz expects."""
    rgb_uint32 = (r << 16) | (g << 8) | b
    return struct.unpack("f", struct.pack("I", rgb_uint32))[0]


def quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> Tuple[Tuple[float, float, float], ...]:
    """
    Convert quaternion to 3x3 rotation matrix.
    Quaternion is assumed normalized (TF typically provides normalized quaternions).
    """
    xx = x * x
    yy = y * y
    zz = z * z
    ww = w * w

    xy = x * y
    xz = x * z
    yz = y * z
    xw = x * w
    yw = y * w
    zw = z * w

    # Standard quaternion -> rotation matrix
    r00 = ww + xx - yy - zz
    r01 = 2.0 * (xy - zw)
    r02 = 2.0 * (xz + yw)

    r10 = 2.0 * (xy + zw)
    r11 = ww - xx + yy - zz
    r12 = 2.0 * (yz - xw)

    r20 = 2.0 * (xz - yw)
    r21 = 2.0 * (yz + xw)
    r22 = ww - xx - yy + zz

    return ((r00, r01, r02),
            (r10, r11, r12),
            (r20, r21, r22))


class PointCloudColorizer(Node):
    def __init__(self):
        super().__init__("pointcloud_colorizer_fast")

        # --- Parameters you can tune at runtime ---
        self.declare_parameter("stride", 20)            # process 1 of every N points (10-50 typical)
        self.declare_parameter("process_rate_hz", 5.0)  # how often to publish colorized cloud
        self.declare_parameter("log_every_n", 10)       # log timing every N publishes

        self.stride = int(self.get_parameter("stride").value) # type: ignore
        self.process_rate_hz = float(self.get_parameter("process_rate_hz").value) # type: ignore
        self.log_every_n = int(self.get_parameter("log_every_n").value) # type: ignore

        self.bridge = CvBridge()

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latest camera intrinsics + images keyed by TF frame name
        self.camera_models: Dict[str, PinholeCameraModel] = {}
        self.camera_images: Dict[str, object] = {}  # cv image (numpy array)

        # Latest pointcloud storage (we process on a timer)
        self.latest_cloud: Optional[PointCloud2] = None

        # Camera subscribers (image + camera_info, synced)
        for cam_id in range(1, 5):
            img_sub = message_filters.Subscriber(
                self, Image, f"/camera{cam_id}/image_raw", qos_profile=qos_profile_sensor_data
            )
            info_sub = message_filters.Subscriber(
                self, CameraInfo, f"/camera{cam_id}/camera_info", qos_profile=qos_profile_sensor_data
            )
            ts = message_filters.ApproximateTimeSynchronizer([img_sub, info_sub], queue_size=10, slop=0.1)
            ts.registerCallback(self.get_camera_callback(cam_id))

        # Pointcloud subscriber: store latest message only
        self.create_subscription(
            PointCloud2,
            "/ouster/points",
            self.on_pointcloud,
            qos_profile_sensor_data,
        )

        # Publish colorized cloud with sensor QoS (BEST_EFFORT)
        self.colorized_pub = self.create_publisher(
            PointCloud2,
            "/colorized_points",
            qos_profile_sensor_data,
        )

        # Timer: process at fixed rate (prevents backlog / keeps it "real time")
        period = 1.0 / max(self.process_rate_hz, 0.1)
        self.timer = self.create_timer(period, self.process_latest)

        self._publish_count = 0

        self.get_logger().info(
            f"Started pointcloud_colorizer_fast with stride={self.stride}, process_rate_hz={self.process_rate_hz}"
        )

    def get_camera_callback(self, cam_id: int):
        def callback(img_msg: Image, info_msg: CameraInfo):
            try:
                cam_model = PinholeCameraModel()
                cam_model.fromCameraInfo(info_msg)

                cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

                # Your static TF uses: "camera", "camera2", "camera3", "camera4"
                cam_frame = "camera" if cam_id == 1 else f"camera{cam_id}"

                self.camera_models[cam_frame] = cam_model
                self.camera_images[cam_frame] = cv_img
            except Exception as e:
                self.get_logger().warn(f"Camera {cam_id} callback error: {e}")

        return callback

    def on_pointcloud(self, cloud_msg: PointCloud2):
        # Just store latest; timer thread will process it.
        self.latest_cloud = cloud_msg

    def _lookup_cam_tf_rt(
        self,
        cam_frame: str,
        cloud_frame: str,
        cloud_time: Time,
    ) -> Optional[Tuple[Tuple[Tuple[float, float, float], ...], Tuple[float, float, float]]]:
        """
        Lookup transform cloud_frame -> cam_frame at cloud_time.
        Return (R, t) where:
          - R is 3x3 rotation matrix
          - t is translation (tx, ty, tz)
        """
        try:
            tf = self.tf_buffer.lookup_transform(cam_frame, cloud_frame, cloud_time)
            q = tf.transform.rotation
            t = tf.transform.translation
            R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
            return (R, (t.x, t.y, t.z))
        except Exception:
            return None

    def process_latest(self):
        # Need camera intrinsics at least
        if not self.camera_models:
            return

        cloud_msg = self.latest_cloud
        if cloud_msg is None:
            return

        # Optional: clear latest_cloud so we don't reprocess same msg if no new one arrives
        self.latest_cloud = None

        t0 = time.perf_counter()

        cloud_time = Time.from_msg(cloud_msg.header.stamp)
        cloud_frame = cloud_msg.header.frame_id

        # Precompute per-camera (R,t) once per processing cycle
        cam_rt: Dict[str, Tuple[Tuple[Tuple[float, float, float], ...], Tuple[float, float, float]]] = {}
        for cam_frame in self.camera_models.keys():
            if self.camera_images.get(cam_frame) is None:
                continue
            rt = self._lookup_cam_tf_rt(cam_frame, cloud_frame, cloud_time)
            if rt is not None:
                cam_rt[cam_frame] = rt

        if not cam_rt:
            # No transforms available (TF not ready / wrong frames)
            return

        colored_points = []

        # Iterate points; downsample with stride
        for idx, (x, y, z) in enumerate(
            point_cloud2.read_points(cloud_msg, field_names=["x", "y", "z"], skip_nans=True)
        ):
            if self.stride > 1 and (idx % self.stride) != 0:
                continue

            best_rgb = None

            # Try each camera until one sees it
            for cam_frame, cam_model in self.camera_models.items():
                img = self.camera_images.get(cam_frame)
                rt = cam_rt.get(cam_frame)
                if img is None or rt is None:
                    continue

                R, t = rt
                tx, ty, tz = t

                # Transform point into camera frame: p_cam = R * p + t
                # p = (x, y, z) in cloud frame
                X = R[0][0] * x + R[0][1] * y + R[0][2] * z + tx
                Y = R[1][0] * x + R[1][1] * y + R[1][2] * z + ty
                Z = R[2][0] * x + R[2][1] * y + R[2][2] * z + tz

                # Reject points behind camera
                if Z <= 0.0:
                    continue

                try:
                    u, v = cam_model.project3dToPixel((X, Y, Z))
                except Exception:
                    continue

                u_i, v_i = int(u), int(v)

                if 0 <= u_i < img.shape[1] and 0 <= v_i < img.shape[0]: # type: ignore
                    b, g, r = img[v_i, u_i]  # type: ignore # OpenCV BGR
                    best_rgb = (int(r), int(g), int(b))
                    break

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

        self._publish_count += 1
        if self.log_every_n > 0 and (self._publish_count % self.log_every_n) == 0:
            dt = time.perf_counter() - t0
            self.get_logger().info(
                f"Published {len(colored_points)} colored pts (stride={self.stride}) in {dt:.3f}s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudColorizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
