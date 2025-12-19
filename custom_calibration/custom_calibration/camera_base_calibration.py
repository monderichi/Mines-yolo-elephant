#!/usr/bin/env python3
"""
Robust Camera-to-Base Calibration using ArUco marker on end-effector.
Features: Sub-pixel refinement, outlier rejection, consistency checks.
"""

import os
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def quaternion_from_matrix(matrix):
    """Return quaternion [x, y, z, w] from 4x4 transformation matrix."""
    M = np.array(matrix, dtype=np.float64)[:4, :4]
    q = np.empty((4,))
    t = np.trace(M[:3, :3])
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        q[3] = 0.25 / s
        q[0] = (M[2, 1] - M[1, 2]) * s
        q[1] = (M[0, 2] - M[2, 0]) * s
        q[2] = (M[1, 0] - M[0, 1]) * s
    else:
        if M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
            s = 2.0 * math.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2])
            q[3] = (M[2, 1] - M[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (M[0, 1] + M[1, 0]) / s
            q[2] = (M[0, 2] + M[2, 0]) / s
        elif M[1, 1] > M[2, 2]:
            s = 2.0 * math.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2])
            q[3] = (M[0, 2] - M[2, 0]) / s
            q[0] = (M[0, 1] + M[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (M[1, 2] + M[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1])
            q[3] = (M[1, 0] - M[0, 1]) / s
            q[0] = (M[0, 2] + M[2, 0]) / s
            q[1] = (M[1, 2] + M[2, 1]) / s
            q[2] = 0.25 * s
    return q / np.linalg.norm(q)


def matrix_from_quaternion(q):
    """Return 3x3 rotation matrix from quaternion [x, y, z, w]."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def euler_from_matrix(R):
    """Return Euler angles (roll, pitch, yaw) from rotation matrix."""
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    return np.array([roll, pitch, yaw])


class RobustCalibrationNode(Node):
    def __init__(self):
        super().__init__('robust_calibration')

        # Parameters
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.05)  # 5cm
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'link6')
        self.declare_parameter('num_samples', 50)  # Collect more for robustness
        self.declare_parameter('max_translation_std', 0.05)  # 5cm std threshold

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.num_samples = self.get_parameter('num_samples').value
        self.max_std = self.get_parameter('max_translation_std').value

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.samples = []
        self.calibrating = True

        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 10)

        self.get_logger().info(f'Robust Calibration started. Collecting {self.num_samples} samples...')
        self.get_logger().info(f'Marker ID: {self.marker_id}, Size: {self.marker_size}m')
        self.get_logger().info('KEEP THE ARM COMPLETELY STILL!')

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def get_marker_corners(self):
        """Define 3D corners of marker in marker frame (centered)."""
        half = self.marker_size / 2.0
        return np.array([
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

    def image_cb(self, msg):
        if not self.calibrating or self.camera_matrix is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is None:
            return

        for i, mid in enumerate(ids.flatten()):
            if mid != self.marker_id:
                continue

            # Sub-pixel corner refinement
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            refined_corners = cv2.cornerSubPix(
                gray, corners[i].reshape(-1, 2), (5, 5), (-1, -1), criteria
            ).reshape(1, 4, 2)

            # Solve PnP with IPPE (Infinitesimal Plane-based Pose Estimation)
            success, rvec, tvec = cv2.solvePnP(
                self.get_marker_corners(),
                refined_corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                continue

            # Refine with Levenberg-Marquardt
            rvec, tvec = cv2.solvePnPRefineLM(
                self.get_marker_corners(),
                refined_corners,
                self.camera_matrix,
                self.dist_coeffs,
                rvec, tvec
            )

            # Build T_cam_marker
            R_cam_marker, _ = cv2.Rodrigues(rvec)
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = R_cam_marker
            T_cam_marker[:3, 3] = tvec.flatten()

            # Get T_base_ee from TF
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.base_frame, self.ee_frame, rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}')
                return

            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation

            T_base_ee = np.eye(4)
            T_base_ee[:3, :3] = matrix_from_quaternion([q.x, q.y, q.z, q.w])
            T_base_ee[:3, 3] = [t.x, t.y, t.z]

            # T_base_cam = T_base_ee @ inv(T_cam_marker)
            # Assumes marker is rigidly attached to EE (T_base_marker ≈ T_base_ee)
            T_base_cam = T_base_ee @ np.linalg.inv(T_cam_marker)

            # Debug: Print both transforms for first few samples
            if len(self.samples) < 3:
                ee_pos = T_base_ee[:3, 3]
                marker_pos = T_cam_marker[:3, 3]
                self.get_logger().info(f'DEBUG T_base_ee (EE in Base from MoveIt FK): [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]')
                self.get_logger().info(f'DEBUG T_cam_marker (Marker in Cam from ArUco): [{marker_pos[0]:.3f}, {marker_pos[1]:.3f}, {marker_pos[2]:.3f}]')

            self.samples.append(T_base_cam)
            count = len(self.samples)
            xyz = T_base_cam[:3, 3]
            self.get_logger().info(f'Sample {count}/{self.num_samples}: [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]')

            if count >= self.num_samples:
                self.finish_calibration()
            return

    def finish_calibration(self):
        self.calibrating = False
        self.get_logger().info('Processing samples with outlier rejection...')

        # Extract translations
        translations = np.array([T[:3, 3] for T in self.samples])

        # RANSAC-style outlier rejection using median absolute deviation
        median = np.median(translations, axis=0)
        mad = np.median(np.abs(translations - median), axis=0)
        mad_threshold = 2.5  # Reject if > 2.5 MAD from median

        inliers = []
        for i, T in enumerate(self.samples):
            t = T[:3, 3]
            deviation = np.abs(t - median) / (mad + 1e-6)
            if np.all(deviation < mad_threshold):
                inliers.append(T)

        self.get_logger().info(f'Kept {len(inliers)}/{len(self.samples)} inliers after outlier rejection')

        if len(inliers) < 5:
            self.get_logger().error('Too few inliers! Calibration failed.')
            raise SystemExit

        # Average inliers
        inlier_translations = np.array([T[:3, 3] for T in inliers])
        avg_t = np.mean(inlier_translations, axis=0)

        # Average quaternions (normalized mean)
        qs = np.array([quaternion_from_matrix(T) for T in inliers])
        avg_q = np.mean(qs, axis=0)
        avg_q /= np.linalg.norm(avg_q)

        # Check consistency
        std_t = np.std(inlier_translations, axis=0)
        self.get_logger().info(f'Translation std: [{std_t[0]:.4f}, {std_t[1]:.4f}, {std_t[2]:.4f}]')

        if np.any(std_t > self.max_std):
            self.get_logger().warn(f'High variance detected! Results may be unreliable.')

        # Build final transform
        T_base_cam = np.eye(4)
        T_base_cam[:3, :3] = matrix_from_quaternion(avg_q)
        T_base_cam[:3, 3] = avg_t

        # Compute inverse
        T_cam_base = np.linalg.inv(T_base_cam)
        inv_t = T_cam_base[:3, 3]
        inv_q = quaternion_from_matrix(T_cam_base)

        # RPY
        rpy_base_cam = euler_from_matrix(T_base_cam[:3, :3])
        rpy_cam_base = euler_from_matrix(T_cam_base[:3, :3])

        # Save
        out_path = 'camera_calibration_result.yaml'
        with open(out_path, 'w') as f:
            f.write("# Robust Calibration Result (with outlier rejection)\n")
            f.write(f"# Inliers: {len(inliers)}/{len(self.samples)}\n")
            f.write("camera_in_base:\n")
            f.write(f"  xyz: [{avg_t[0]:.6f}, {avg_t[1]:.6f}, {avg_t[2]:.6f}]\n")
            f.write(f"  quat_xyzw: [{avg_q[0]:.6f}, {avg_q[1]:.6f}, {avg_q[2]:.6f}, {avg_q[3]:.6f}]\n")
            f.write(f"  rpy_rad: [{rpy_base_cam[0]:.6f}, {rpy_base_cam[1]:.6f}, {rpy_base_cam[2]:.6f}]\n")
            f.write("\nbase_in_camera:\n")
            f.write(f"  xyz: [{inv_t[0]:.6f}, {inv_t[1]:.6f}, {inv_t[2]:.6f}]\n")
            f.write(f"  quat_xyzw: [{inv_q[0]:.6f}, {inv_q[1]:.6f}, {inv_q[2]:.6f}, {inv_q[3]:.6f}]\n")
            f.write(f"  rpy_rad: [{rpy_cam_base[0]:.6f}, {rpy_cam_base[1]:.6f}, {rpy_cam_base[2]:.6f}]\n")
            f.write(f"\nsamples: {len(self.samples)}\n")
            f.write(f"inliers: {len(inliers)}\n")

        self.get_logger().info(f'Calibration saved to {os.path.abspath(out_path)}')
        self.get_logger().info(f'Camera in Base: xyz={avg_t}, rpy={rpy_base_cam}')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = RobustCalibrationNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
