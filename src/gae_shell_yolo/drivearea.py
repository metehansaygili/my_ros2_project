import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

import tf2_ros
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from geometry_msgs.msg import PointStamped, Quaternion
import math


class DrivableAreaNode(Node):
    def __init__(self):
        super().__init__('drivable_area_node')

        # --- MODEL VE AYARLAR ---
        # Model yolunu kendi sisteminize göre güncelleyin
        self.model_path = '/home/otonom/runs/segment/train10/weights/best.pt'
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # Slice (Dilimleme) Ayarları
        self.ROI_Y_START_PERCENT = 0.35
        self.ROI_Y_END_PERCENT = 0.95
        self.NUM_SLICES = 35
        self.CONFIDENCE_THRESHOLD = 0.30
        
        # --- Polygon publish settings ---
        self.POLY_DOWNSAMPLE_STEP = 8         
        self.POLY_MIN_POINTS = 20             
        self.POLY_APPROX_EPS_PX = 3.0         

        # --- Camera intrinsics ---
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.width = 1280
        self.height = 720
        self.is_initialized = False

        # --- Centerline Parameters ---
        self.prev_poly_coeffs = None 
        self.POLY_ALPHA = 0.6        

        # --- TF Buffer ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(
            Image,
            '/zed2_left_camera/image_raw',
            self.image_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            "/zed_fixed/camera_info",
            self.camera_info_cb,
            10
        )

        # Publishers
        self.publisher_image_ = self.create_publisher(Image, '/drivable_area/image_masked', 10)
        self.path_base_pub = self.create_publisher(Path, "/drivable_area/centerline_base_link", 10)
        self.road_poly_base_pub = self.create_publisher(PolygonStamped, "/drivable_area/road_polygon_base_link", 10)

        self.get_logger().info("Drivable Area Node Started with Noise Filtering Logic.")

    def camera_info_cb(self, msg: CameraInfo):
        if not self.is_initialized:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.width = msg.width
            self.height = msg.height
            self.is_initialized = True
            self.get_logger().info(
                f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
            )

    def get_quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    def pixel_to_base_ground(self, u, v, tf_cam_to_base):
        """
        Pixel (u,v) -> Base Link (x,y,0) dönüşümü
        """
        if not self.is_initialized:
            return None

        # 1. Kamera koordinatlarına (Ray) çevir
        nx = (u - self.cx) / self.fx
        ny = (v - self.cy) / self.fy
        nz = 1.0
        
        d = math.sqrt(nx*nx + ny*ny + nz*nz)
        nx, ny, nz = nx/d, ny/d, nz/d

        # 2. Ray'i base_link frame'ine taşı
        cam_origin = tf_cam_to_base.transform.translation
        p0 = np.array([cam_origin.x, cam_origin.y, cam_origin.z])

        p_ray_cam = PointStamped()
        p_ray_cam.point.x = float(nx)
        p_ray_cam.point.y = float(ny)
        p_ray_cam.point.z = float(nz)
        
        p_ray_base_msg = do_transform_point(p_ray_cam, tf_cam_to_base)
        p1 = np.array([p_ray_base_msg.point.x, p_ray_base_msg.point.y, p_ray_base_msg.point.z])
        
        vec_base = p1 - p0
        norm_v = np.linalg.norm(vec_base)
        if norm_v < 1e-6:
            return None
        vec_base /= norm_v
        
        # 3. Z=0 düzlemi ile kesiştir
        if abs(vec_base[2]) < 1e-4:
            return None 
            
        t = -p0[2] / vec_base[2]
        
        if t < 0:
            return None # Arkada kaldı
            
        p_ground = p0 + t * vec_base
        
        return (p_ground[0], p_ground[1], 0.0)

    def smooth_path_base(self, points_3d):
        """
        Noktalar azsa veya fit edilemezse düz çizgi olarak birleştirir.
        """
        path = []
        if not points_3d:
            return path
            
        for i in range(len(points_3d)):
            p = points_3d[i]
            x, y, z = p
            
            if i < len(points_3d) - 1:
                p_next = points_3d[i+1]
                dx = p_next[0] - x
                dy = p_next[1] - y
                yaw = math.atan2(dy, dx)
            elif i > 0:
                p_prev = points_3d[i-1]
                dx = x - p_prev[0]
                dy = y - p_prev[1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
                
            q = self.get_quaternion_from_yaw(yaw)
            path.append((x, y, z, q))
            
        return path

    def get_poly_path(self, points_3d):
        """
        2. derece polinom fit yaparak yolu pürüzsüzleştirir.
        """
        if len(points_3d) < 3:
            return self.smooth_path_base(points_3d)

        x_vals = np.array([p[0] for p in points_3d])
        y_vals = np.array([p[1] for p in points_3d])

        try:
             # Base_link'te x ileri eksen olduğu için x'e göre y fit ediyoruz
             coeffs = np.polyfit(x_vals, y_vals, 2)
             
             if self.prev_poly_coeffs is not None:
                 coeffs = self.POLY_ALPHA * coeffs + (1.0 - self.POLY_ALPHA) * self.prev_poly_coeffs
             
             self.prev_poly_coeffs = coeffs
             p = np.poly1d(coeffs)
        except:
             return self.smooth_path_base(points_3d)

        start_x = 0.0 # Araç merkezi
        end_x = np.max(x_vals) 
        
        # 0.5m aralıklarla nokta üret
        num_steps = int((end_x - start_x) / 0.5) 
        num_steps = max(num_steps, 5)
        
        xs = np.linspace(start_x, end_x, num_steps)
        ys = p(xs)
        
        path = []
        for i in range(len(xs)):
             deriv = 2*coeffs[0]*xs[i] + coeffs[1]
             yaw = math.atan(deriv)
             q = self.get_quaternion_from_yaw(yaw)
             path.append((xs[i], ys[i], 0.0, q))
             
        return path

    def publish_polygon_like_transformed_path(self, largest_contour, msg_header):
        """
        Algılanan yolun sınırlarını PolygonStamped olarak yayınlar (Rviz için).
        """
        if not self.is_initialized or largest_contour is None:
            return

        contour = largest_contour
        if self.POLY_APPROX_EPS_PX and self.POLY_APPROX_EPS_PX > 0.0:
            contour = cv2.approxPolyDP(largest_contour, self.POLY_APPROX_EPS_PX, True)

        contour = contour.reshape(-1, 2)
        if contour.shape[0] < self.POLY_MIN_POINTS:
            return

        contour = contour[::max(1, int(self.POLY_DOWNSAMPLE_STEP))]

        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                "base_link",
                "zed2_left_camera_optical_frame",
                rclpy.time.Time()
            )
        except Exception:
            return

        poly_base = PolygonStamped()
        poly_base.header.stamp = msg_header.stamp
        poly_base.header.frame_id = "base_link"

        for (u, v) in contour:
            p3_base = self.pixel_to_base_ground(int(u), int(v), tf_cam_to_base)
            if not p3_base:
                continue

            pb = Point32()
            pb.x = float(p3_base[0])
            pb.y = float(p3_base[1])
            pb.z = float(p3_base[2])
            poly_base.polygon.points.append(pb)

        if len(poly_base.polygon.points) >= 3:
            self.road_poly_base_pub.publish(poly_base)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = cv_image.shape[:2]
        except:
            return

        # 1. YOLO Tahmini
        results = self.model.predict(cv_image, verbose=False, conf=self.CONFIDENCE_THRESHOLD)
        if not results or results[0].masks is None:
            return

        # 2. Maske Oluşturma
        full_mask = np.zeros((h, w), dtype=np.uint8)
        for i, cls_id in enumerate(results[0].boxes.cls):
            if int(cls_id) == 0: # Sadece yol sınıfı
                points = np.array(results[0].masks.xy[i], dtype=np.int32)
                cv2.drawContours(full_mask, [points], -1, 255, thickness=cv2.FILLED)

        # 3. En büyük alanı bul (İlk ve kaba temizlik)
        contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest_contour = max(contours, key=cv2.contourArea)
        clean_road_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(clean_road_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

        # Polygonu yayınla
        self.publish_polygon_like_transformed_path(largest_contour, msg.header)

        out_img = cv_image.copy()
        
        # --- DİLİMLEME (SLICING) PARAMS ---
        roi_y_start = int(h * self.ROI_Y_START_PERCENT)
        roi_y_end = int(h * self.ROI_Y_END_PERCENT)
        slice_height = max(1, int((roi_y_end - roi_y_start) / self.NUM_SLICES))

        raw_centers = []

        # Parçalar arası minimum boşluk (pixel)
        # Eğer yanyana beyaz pikseller arasında 20px'den fazla boşluk varsa
        # bunlar ayrı parçalar (yol ve gürültü) olarak kabul edilir.
        SEGMENT_GAP_THRESHOLD = 20 

        # Aşağıdan yukarıya doğru tarama
        for i in range(self.NUM_SLICES):
            y_bottom = roi_y_end - (i * slice_height)
            y_top = y_bottom - slice_height
            if y_top < roi_y_start:
                break

            slice_roi = clean_road_mask[y_top:y_bottom, :]
            
            # Bu dilimdeki tüm beyaz piksellerin X koordinatlarını al
            # [1] indeksi sütunları (X eksenini) verir.
            pixel_x_indices = np.where(slice_roi == 255)[1]
            
            # Koordinatları sırala (genelde sıralı gelir ama garanti olsun)
            pixel_x_indices = np.sort(pixel_x_indices)

            if len(pixel_x_indices) > 0:
                # --- YENİ ALAN FİLTRESİ (GÜRÜLTÜ ENGELLEME) ---
                
                # 1. Pikseller arasındaki farkları bul (türev al)
                # Örnek: [100, 101, 102,  150, 151] -> Farklar: [1, 1, 48, 1]
                diffs = np.diff(pixel_x_indices)
                
                # 2. Farkın eşik değerden büyük olduğu yerleri bul (Kopukluk noktaları)
                split_indices = np.where(diffs > SEGMENT_GAP_THRESHOLD)[0] + 1
                
                # 3. Diziyi kopukluk noktalarından böl
                segments = np.split(pixel_x_indices, split_indices)
                
                # 4. En uzun (en geniş) parçayı seç -> Bu ANA YOLDUR.
                # Sağdaki veya soldaki küçük gürültü kısa olacağı için elenir.
                main_segment = max(segments, key=len)
                
                # Sadece ana segmentin sınırlarını al
                curr_left = int(main_segment[0])
                curr_right = int(main_segment[-1])
                
                center_x = int((curr_left + curr_right) / 2)
                center_y = int((y_top + y_bottom) / 2)

                # Sınır kontrolü (Görüntü dışına taşmayı engelle)
                center_x = max(0, min(w - 1, center_x))
                
                raw_centers.append((center_x, center_y))
                
                # Görselleştirme (Debug)
                # Yeşil noktalar: Algoritmanın "Yol" olarak kabul ettiği sınırlar
                cv2.circle(out_img, (curr_left, center_y), 3, (0, 255, 0), -1)
                cv2.circle(out_img, (curr_right, center_y), 3, (0, 255, 0), -1)
                # Mavi nokta: Hesaplanan merkez
                cv2.circle(out_img, (center_x, center_y), 4, (255, 0, 0), -1)

        # --- TF & Path Publishing ---
        tf_cam_to_base = None
        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                "base_link",
                "zed2_left_camera_optical_frame",
                rclpy.time.Time() 
            )
        except Exception:
            pass

        if tf_cam_to_base and len(raw_centers) >= 3:
            path_points_base = []
            
            for (cx, cy) in raw_centers:
                p_center = self.pixel_to_base_ground(cx, cy, tf_cam_to_base)
                if p_center:
                    path_points_base.append(p_center)
            
            # Polinom fit yaparak yolu pürüzsüzleştir
            smoothed_base = self.get_poly_path(path_points_base)
            
            if len(smoothed_base) > 0:
                path_msg = Path()
                path_msg.header = msg.header
                path_msg.header.frame_id = "base_link"
                
                for (px, py, pz, pq) in smoothed_base:
                    pose = PoseStamped()
                    pose.header = msg.header
                    pose.header.frame_id = "base_link"
                    pose.pose.position.x = px
                    pose.pose.position.y = py
                    pose.pose.position.z = pz
                    pose.pose.orientation = pq
                    path_msg.poses.append(pose)
                    
                self.path_base_pub.publish(path_msg)

        # Çizilen çizgiyi görselleştir
        if raw_centers:
             pts_np = np.array([raw_centers], dtype=np.int32)
             pts_np = pts_np.reshape((-1, 1, 2))
             cv2.polylines(out_img, [pts_np], False, (0, 255, 0), 2)

        # Orijinal maskeyi hafifçe overlay yap
        overlay = out_img.copy()
        cv2.drawContours(overlay, [largest_contour], -1, (0, 255, 0), -1)
        out_img = cv2.addWeighted(overlay, 0.2, out_img, 0.8, 0)

        try:
            self.publisher_image_.publish(self.bridge.cv2_to_imgmsg(out_img, "bgr8"))
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DrivableAreaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()