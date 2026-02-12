#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray


def reorder_corners_tl_tr_br_bl(corners):
    """
    Reorder 4 corners (x,y) into:
      top-left, top-right, bottom-right, bottom-left
    This makes solvePnP stable regardless of input order.
    """
    pts = np.array([[c.x, c.y] for c in corners], dtype=np.float64)

    # sort by y (top 2 have smallest y)
    idx = np.argsort(pts[:, 1])
    top = pts[idx[:2], :]
    bottom = pts[idx[2:], :]

    # sort each pair by x
    top = top[np.argsort(top[:, 0]), :]
    bottom = bottom[np.argsort(bottom[:, 0]), :]

    tl = top[0]
    tr = top[1]
    bl = bottom[0]
    br = bottom[1]

    return np.array([tl, tr, br, bl], dtype=np.float64)


def rotmat_to_quat_xyzw(R):
    """
    Convert 3x3 rotation matrix to quaternion (x,y,z,w).
    """
    tr = np.trace(R)
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        # find largest diagonal element
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    # normalize for safety
    q /= np.linalg.norm(q) + 1e-12
    return q


class TagPoseFromDetections(Node):
    def __init__(self):
        super().__init__("tag_pose_from_detections")

        # ---- Parameters ----
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("camera_info_topic", "/camera/fisheye2/camera_info")
        self.declare_parameter("tag_size_m", 0.10)     # <-- MUST match your physical tag size!
        self.declare_parameter("tag_id", -1)          # -1 = publish all tags; else only this id

        self.detections_topic = self.get_parameter("detections_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.tag_id = int(self.get_parameter("tag_id").value)

        self.K = None
        self.D = None

        # ---- Subscribers ----
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self.detections_cb,
            qos_profile_sensor_data
        )

        # ---- Publisher ----
        self.pose_pub = self.create_publisher(PoseStamped, "/tag_pose", 10)

        self.get_logger().info(
            f"Listening: {self.detections_topic} and {self.camera_info_topic}. "
            f"Publishing: /tag_pose. tag_size_m={self.tag_size_m}, tag_id={self.tag_id}"
        )

    def camera_info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        # Distortion can be empty for rectified streams; OpenCV accepts empty/zeros.
        if len(msg.d) > 0:
            D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            D = np.zeros((5, 1), dtype=np.float64)

        self.K = K
        self.D = D

    def detections_cb(self, msg: AprilTagDetectionArray):
        if self.K is None:
            self.get_logger().warn("No CameraInfo received yet; cannot compute pose.")
            return

        if self.tag_size_m <= 0.0:
            self.get_logger().error("tag_size_m must be > 0")
            return

        s = self.tag_size_m
        half = s / 2.0

        # 3D tag corners in tag frame (z=0 plane), order: TL, TR, BR, BL
        obj_pts = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        # Choose best flag available
        pnp_flag = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE

        for det in msg.detections:
            if (self.tag_id != -1) and (det.id != self.tag_id):
                continue

            if len(det.corners) != 4:
                self.get_logger().warn(f"Detection id={det.id} has {len(det.corners)} corners, expected 4.")
                continue

            img_pts = reorder_corners_tl_tr_br_bl(det.corners)

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts,
                img_pts,
                self.K,
                self.D,
                flags=pnp_flag
            )

            if not ok:
                self.get_logger().warn(f"solvePnP failed for tag id={det.id}")
                continue

            # tvec is tag origin position in camera coordinates (OpenCV camera frame)
            t = tvec.reshape(3)
            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotmat_to_quat_xyzw(R)

            out = PoseStamped()
            out.header = msg.header  # frame_id = camera frame from detections header
            out.pose.position.x = float(t[0])
            out.pose.position.y = float(t[1])
            out.pose.position.z = float(t[2])
            out.pose.orientation.x = float(qx)
            out.pose.orientation.y = float(qy)
            out.pose.orientation.z = float(qz)
            out.pose.orientation.w = float(qw)

            self.pose_pub.publish(out)

            self.get_logger().info(
                f"tag id={det.id} XYZ = [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m "
                f"in frame '{msg.header.frame_id}'"
            )


def main():
    rclpy.init()
    node = TagPoseFromDetections()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
