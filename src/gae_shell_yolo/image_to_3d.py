import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import Path

import numpy as np
import cv2
import cupy as cp  # GPU işlemleri

import tf2_ros
import math

class DrivableArea3D(Node):
    def __init__(self):
        super().__init__("drivable_area_3d")

        # ---------------------------------------------------------
        # 1. PARAMETRE TANIMLAMALARI (Config'den okunacak)
        # ---------------------------------------------------------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topic_camera_info', '/zed_fixed/camera_info'),
                ('topic_polygon', '/drivable_area/polygon'),
                ('topic_lidar', '/lidar_front/points_in'),
                ('topic_path_in', '/detected_path'),
                ('topic_pc_out', '/drivable_area/pointcloud3d'),
                ('topic_path_out', '/drivable_area/transformed_path'),
                ('frame_camera', 'zed2_left_camera_optical_frame'),
                ('frame_lidar', 'lidar_front'),
                ('frame_base', 'base_link'),
                ('max_path_distance', 30.0)
            ]
        )

        # Parametreleri değişkenlere ata
        self.topic_cam_info = self.get_parameter('topic_camera_info').value
        self.topic_poly = self.get_parameter('topic_polygon').value
        self.topic_lidar = self.get_parameter('topic_lidar').value
        self.topic_path_in = self.get_parameter('topic_path_in').value
        self.topic_pc_out = self.get_parameter('topic_pc_out').value
        self.topic_path_out = self.get_parameter('topic_path_out').value
        
        self.frame_cam = self.get_parameter('frame_camera').value
        self.frame_lidar = self.get_parameter('frame_lidar').value
        self.frame_base = self.get_parameter('frame_base').value
        self.max_dist = self.get_parameter('max_path_distance').value

        # ---------------------------------------------------------
        
        # --- Camera intrinsics ---
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.is_initialized = False

        # --- Drivable Area Polygon ---
        self.drivable_poly_np = None

        # --- TF Buffer ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers (Parametrelerden gelen topic isimleri ile)
        self.create_subscription(CameraInfo, self.topic_cam_info, self.camera_info_cb, 10)
        self.create_subscription(PolygonStamped, self.topic_poly, self.polygon_cb, 10)
        self.create_subscription(PointCloud2, self.topic_lidar, self.lidar_cb, 10)
        self.create_subscription(Path, self.topic_path_in, self.path_2d_cb, 10)

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, self.topic_pc_out, 5)
        self.path_pub = self.create_publisher(Path, self.topic_path_out, 5)

        self.get_logger().info(f"3D Drivable Area Node (GPU) Started.")
        self.get_logger().info(f"Listening LiDAR: {self.topic_lidar} | Frame: {self.frame_lidar}")

        self.pixel_centerline = None

    def path_2d_cb(self, msg: Path):
        pts = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            pts.append([x, y])
        if len(pts) >= 2:
            self.pixel_centerline = np.array(pts, dtype=np.float32)
        else:
            self.pixel_centerline = None

    def camera_info_cb(self, msg: CameraInfo):
        if not self.is_initialized:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.is_initialized = True
            self.get_logger().info("Camera intrinsics initialized.")

    def polygon_cb(self, msg: PolygonStamped):
        pts = []
        for p in msg.polygon.points:
            pts.append([p.x, p.y])
        if len(pts) >= 3:
            self.drivable_poly_np = np.array(pts, dtype=np.int32)
        else:
            self.drivable_poly_np = None

    def lidar_cb(self, msg: PointCloud2):
        if not self.is_initialized or self.drivable_poly_np is None:
            return

        # 1. TF Lookup (Parametrik Frame isimleri ile)
        try:
            # Lidar -> Camera
            T_lidar_to_cam = self.tf_buffer.lookup_transform(self.frame_cam, self.frame_lidar, rclpy.time.Time())
            # Lidar -> Base
            T_lidar_to_base = self.tf_buffer.lookup_transform(self.frame_base, self.frame_lidar, rclpy.time.Time())
            # Camera -> Lidar
            cam_to_lidar = self.tf_buffer.lookup_transform(self.frame_lidar, self.frame_cam, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # 2. PointCloud Verisini GPU'ya Taşıma
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw_list = list(gen)
        if not raw_list:
            return

        raw_arr = np.array(raw_list)
        if raw_arr.dtype.names:
            lidar_points_cpu = np.column_stack((raw_arr['x'], raw_arr['y'], raw_arr['z'])).astype(np.float32)
        else:
            lidar_points_cpu = raw_arr.astype(np.float32)
        
        lidar_points_gpu = cp.asarray(lidar_points_cpu) # Nx3

        # 3. Matrisleri GPU'ya Taşıma
        R_lidar_to_cam = cp.asarray(self.quat_to_mat(T_lidar_to_cam.transform.rotation))
        t_lidar_to_cam = cp.asarray([T_lidar_to_cam.transform.translation.x, T_lidar_to_cam.transform.translation.y, T_lidar_to_cam.transform.translation.z])

        R_lidar_to_base = cp.asarray(self.quat_to_mat(T_lidar_to_base.transform.rotation))
        t_lidar_to_base = cp.asarray([T_lidar_to_base.transform.translation.x, T_lidar_to_base.transform.translation.y, T_lidar_to_base.transform.translation.z])

        cam_to_lidar_R = cp.asarray(self.quat_to_mat(cam_to_lidar.transform.rotation))
        
        # --- BÖLÜM 1: Polygon Kontrolü ---
        p_cam_gpu = cp.dot(lidar_points_gpu, R_lidar_to_cam.T) + t_lidar_to_cam
        mask_z = p_cam_gpu[:, 2] > 0
        
        if not cp.any(mask_z):
            return

        valid_indices = cp.where(mask_z)[0]
        p_cam_gpu_valid = p_cam_gpu[valid_indices]

        X = p_cam_gpu_valid[:, 0]
        Y = p_cam_gpu_valid[:, 1]
        Z = p_cam_gpu_valid[:, 2]

        u_gpu = (self.fx * (X / Z) + self.cx).astype(cp.int32)
        v_gpu = (self.fy * (Y / Z) + self.cy).astype(cp.int32)

        u_cpu = cp.asnumpy(u_gpu)
        v_cpu = cp.asnumpy(v_gpu)

        has_points_in_poly = False
        for i in range(len(u_cpu)):
            if cv2.pointPolygonTest(self.drivable_poly_np, (int(u_cpu[i]), int(v_cpu[i])), False) >= 0:
                has_points_in_poly = True
                break
        
        if not has_points_in_poly:
            return

        # --- BÖLÜM 2: Path Ray Casting ---
        if self.pixel_centerline is None:
            return

        path_points_base_gpu = []

        for (u, v) in self.pixel_centerline:
            x = (u - self.cx) / self.fx
            y = (v - self.cy) / self.fy
            z = 1.0
            
            ray_cam_gpu = cp.array([x, y, z], dtype=cp.float32)
            ray_cam_gpu /= cp.linalg.norm(ray_cam_gpu)

            ray_lidar_gpu = cp.dot(cam_to_lidar_R, ray_cam_gpu)
            proj_gpu = cp.dot(lidar_points_gpu, ray_lidar_gpu)

            mask_proj = proj_gpu > 0
            if not cp.any(mask_proj):
                continue
            
            pts_subset = lidar_points_gpu[mask_proj]
            proj_subset = proj_gpu[mask_proj]

            closest_on_ray = proj_subset[:, None] * ray_lidar_gpu[None, :]
            diff = pts_subset - closest_on_ray
            dist_sq = cp.sum(diff**2, axis=1)

            min_idx = cp.argmin(dist_sq)
            p_lidar_hit = pts_subset[min_idx]

            p_base_gpu = cp.dot(R_lidar_to_base, p_lidar_hit) + t_lidar_to_base
            path_points_base_gpu.append(p_base_gpu)

        path_msg = Path()
        path_msg.header.frame_id = self.frame_base # Parametrik frame

        for p_gpu in path_points_base_gpu:
            p = cp.asnumpy(p_gpu)
            
            pose = PoseStamped()
            pose.header.frame_id = self.frame_base
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.position.z = float(p[2])
            
            dx = abs(p[0])
            dy = abs(p[1])
            distance_sq = dx * dx + dy * dy
            
            # Parametrik max mesafe kontrolü
            if math.sqrt(distance_sq) > self.max_dist:
                continue
                
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def quat_to_mat(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y*y + z*z),     2 * (x*y - z*w),     2 * (x*z + y*w)],
            [2 * (x*y + z*w),         1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w),         2 * (y*z + x*w),     1 - 2 * (x*x + y*y)]
        ], dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = DrivableArea3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()