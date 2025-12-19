#!/usr/bin/env python3
"""Dynamic visual servoing using MoveIt2 IK (pose goals).

Goal:
- Detect ArUco marker (ID 0, 4x4, 30mm by default) in the RealSense D455 image.
- Compute the 3D pose error in the camera frame.
- Convert that error into a small incremental end-effector pose update in the robot base frame.
- Send the updated end-effector pose to MoveIt2 (MoveGroup + ExecuteTrajectory), letting MoveIt solve IK.

This is intended for an eye-to-hand setup where the camera is fixed and the marker is rigidly
attached to the end-effector. The script does *not* require hand-eye calibration to start moving;
it uses the base->camera TF rotation to map camera-frame translation errors into base-frame steps.

Usage:
    ros2 run yolo_realsense visual_servo_ik.py

Notes:
- Requires MoveIt2 action servers: /move_action and /execute_trajectory
- Requires TF frames: base_frame -> camera_frame and base_frame -> ee_frame
"""

import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, MotionPlanRequest, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
import tf2_ros
from tf2_ros import TransformException


def _clamp_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-12 or norm <= max_norm:
        return vec
    return vec * (max_norm / norm)


class VisualServoIK(Node):
    def __init__(self) -> None:
        super().__init__('visual_servo_ik')

        # ---- Parameters ----
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.03)  # meters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'link6')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        # If TF is disconnected (common with external cameras), you can provide the camera pose
        # in the robot base frame here and set use_camera_in_base_params:=True.
        self.declare_parameter('use_camera_in_base_params', False)
        self.declare_parameter('camera_in_base_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('camera_in_base_quat_xyzw', [0.0, 0.0, 0.0, 1.0])

        self.declare_parameter('move_group_name', 'arm')
        self.declare_parameter('target_distance', 0.50)  # meters
        self.declare_parameter('gain', 0.7)
        self.declare_parameter('max_step', 0.02)  # meters per command
        self.declare_parameter('pos_tolerance', 0.01)  # meters (MoveIt constraint sphere radius)
        self.declare_parameter('orient_tolerance', 0.25)  # radians
        self.declare_parameter('deadband', 0.005)  # meters in camera frame
        self.declare_parameter('control_rate_hz', 5.0)
        self.declare_parameter('planning_time', 3.0)
        self.declare_parameter('plan_attempts', 5)

        aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.marker_id = int(self.get_parameter('marker_id').get_parameter_value().integer_value)
        self.marker_size = float(self.get_parameter('marker_size').get_parameter_value().double_value)
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.use_camera_in_base_params = bool(
            self.get_parameter('use_camera_in_base_params').get_parameter_value().bool_value
        )
        self.camera_in_base_xyz = np.array(
            self.get_parameter('camera_in_base_xyz').get_parameter_value().double_array_value,
            dtype=np.float64,
        )
        self.camera_in_base_quat_xyzw = np.array(
            self.get_parameter('camera_in_base_quat_xyzw').get_parameter_value().double_array_value,
            dtype=np.float64,
        )

        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.target_distance = float(self.get_parameter('target_distance').get_parameter_value().double_value)
        self.gain = float(self.get_parameter('gain').get_parameter_value().double_value)
        self.max_step = float(self.get_parameter('max_step').get_parameter_value().double_value)
        self.pos_tolerance = float(self.get_parameter('pos_tolerance').get_parameter_value().double_value)
        self.orient_tolerance = float(self.get_parameter('orient_tolerance').get_parameter_value().double_value)
        self.deadband = float(self.get_parameter('deadband').get_parameter_value().double_value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').get_parameter_value().double_value)
        self.planning_time = float(self.get_parameter('planning_time').get_parameter_value().double_value)
        self.plan_attempts = int(self.get_parameter('plan_attempts').get_parameter_value().integer_value)

        # ---- ArUco detector ----
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        }
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dicts.get(aruco_dict_name, cv2.aruco.DICT_4X4_50))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # ---- Camera ----
        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.latest_tvec: Optional[np.ndarray] = None
        self.latest_stamp = None
        self.marker_visible = False
        self.last_detection_time = self.get_clock().now()

        self._last_tf_warning_time = 0.0

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # ---- TF2 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- MoveIt actions ----
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # ---- Control loop ----
        period = 1.0 / max(control_rate_hz, 0.1)
        self.create_timer(period, self.control_loop)

        self.get_logger().info('VisualServoIK ready')
        self.get_logger().info(
            f'ArUco={aruco_dict_name} id={self.marker_id} size={self.marker_size}m | '
            f'base={self.base_frame} ee={self.ee_frame} cam={self.camera_frame}'
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg: Image) -> None:
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is None:
            self.marker_visible = False
            self.latest_tvec = None
            return

        for i, mid in enumerate(ids.flatten()):
            if int(mid) != self.marker_id:
                continue

            marker_corners = corners[i][0]
            half = self.marker_size / 2.0
            obj_points = np.array(
                [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
                dtype=np.float32,
            )

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                marker_corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            if not success:
                self.marker_visible = False
                self.latest_tvec = None
                return

            self.latest_tvec = tvec.flatten().astype(np.float64)
            self.latest_stamp = msg.header.stamp
            self.marker_visible = True
            self.last_detection_time = self.get_clock().now()
            return

        self.marker_visible = False
        self.latest_tvec = None

    def lookup_T(self, target_frame: str, source_frame: str) -> Optional[np.ndarray]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )

            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            # xyzw
            x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)

            # Quaternion -> rotation matrix
            Rm = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ],
                dtype=np.float64,
            )

            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = Rm
            T[:3, 3] = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)
            return T
        except TransformException as e:
            now_s = time.time()
            if now_s - self._last_tf_warning_time > 5.0:
                self._last_tf_warning_time = now_s
                self.get_logger().warn(
                    f"TF lookup failed ({target_frame} <- {source_frame}): {e}"
                )
                self.get_logger().warn(
                    'TF trees appear disconnected. For an external fixed camera, you must publish a'
                    ' static transform base->camera (from calibration), or set'
                    ' use_camera_in_base_params:=True and provide camera_in_base_xyz / camera_in_base_quat_xyzw.'
                )
            return None

    def T_base_cam(self) -> Optional[np.ndarray]:
        """Return base<-camera transform either from TF or from parameters."""
        if self.use_camera_in_base_params:
            if self.camera_in_base_xyz.shape[0] != 3 or self.camera_in_base_quat_xyzw.shape[0] != 4:
                self.get_logger().error('camera_in_base_xyz must be 3 values and camera_in_base_quat_xyzw 4 values')
                return None

            x, y, z, w = [float(v) for v in self.camera_in_base_quat_xyzw]
            Rm = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ],
                dtype=np.float64,
            )

            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = Rm
            T[:3, 3] = self.camera_in_base_xyz.astype(np.float64)
            return T

        return self.lookup_T(self.base_frame, self.camera_frame)

    def plan_and_execute_pose(self, target_pose: PoseStamped) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveGroup action server not available (/move_action)')
            return False
        if not self._exec_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('ExecuteTrajectory action server not available (/execute_trajectory)')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = self.move_group_name
        goal_msg.request.num_planning_attempts = self.plan_attempts
        goal_msg.request.allowed_planning_time = self.planning_time

        constraints = Constraints()

        pos = PositionConstraint()
        pos.header = target_pose.header
        pos.link_name = self.ee_frame
        pos.target_point_offset.x = 0.0
        pos.target_point_offset.y = 0.0
        pos.target_point_offset.z = 0.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.pos_tolerance]
        pos.constraint_region.primitives.append(primitive)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        pos.weight = 1.0

        orient = OrientationConstraint()
        orient.header = target_pose.header
        orient.link_name = self.ee_frame
        orient.orientation = target_pose.pose.orientation
        orient.absolute_x_axis_tolerance = self.orient_tolerance
        orient.absolute_y_axis_tolerance = self.orient_tolerance
        orient.absolute_z_axis_tolerance = self.orient_tolerance
        orient.weight = 0.5

        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(orient)
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True

        send_goal_future = self._move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Planning goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val != 1:
            self.get_logger().warn(f'Planning failed: error code {result.error_code.val}')
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.planned_trajectory

        exec_future = self._exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)

        exec_handle = exec_future.result()
        if exec_handle is None or not exec_handle.accepted:
            self.get_logger().warn('Execution rejected')
            return False

        exec_result_future = exec_handle.get_result_async()
        timeout = 30.0
        start = time.time()
        while not exec_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().warn('Execution timeout')
                return True

        return True

    def control_loop(self) -> None:
        if not self.marker_visible or self.latest_tvec is None:
            return

        # Stop commanding if marker is stale
        age = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if age > 0.5:
            return

        # Desired marker position in camera frame
        t_des_cam = np.array([0.0, 0.0, self.target_distance], dtype=np.float64)
        t_cur_cam = self.latest_tvec
        err_cam = t_cur_cam - t_des_cam

        # Deadband
        if float(np.linalg.norm(err_cam)) < self.deadband:
            return

        # Marker should move by -err in camera frame
        delta_marker_cam = -err_cam
        delta_marker_cam = self.gain * _clamp_norm(delta_marker_cam, self.max_step)

        # Map camera-frame translation delta into base-frame delta using base<-camera rotation
        T_base_cam = self.T_base_cam()
        if T_base_cam is None:
            return

        R_base_cam = T_base_cam[:3, :3]
        delta_base = R_base_cam @ delta_marker_cam

        # Get current EE pose from TF
        T_base_ee = self.lookup_T(self.base_frame, self.ee_frame)
        if T_base_ee is None:
            return

        ee_pos = T_base_ee[:3, 3]
        ee_pos_next = ee_pos + delta_base

        # Keep current orientation
        # Extract quaternion from T_base_ee
        Rm = T_base_ee[:3, :3]
        qw = np.sqrt(max(0.0, 1.0 + Rm[0, 0] + Rm[1, 1] + Rm[2, 2])) / 2.0
        qx = (Rm[2, 1] - Rm[1, 2]) / (4.0 * qw + 1e-12)
        qy = (Rm[0, 2] - Rm[2, 0]) / (4.0 * qw + 1e-12)
        qz = (Rm[1, 0] - Rm[0, 1]) / (4.0 * qw + 1e-12)

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(ee_pos_next[0])
        pose.pose.position.y = float(ee_pos_next[1])
        pose.pose.position.z = float(ee_pos_next[2])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        # Plan+execute a small step
        self.plan_and_execute_pose(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualServoIK()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Avoid noisy shutdown errors on Ctrl-C in some environments.
            pass


if __name__ == '__main__':
    main()
