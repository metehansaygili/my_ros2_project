import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class CameraFrameFixer(Node):
    def __init__(self):
        super().__init__("camera_frame_fixer")

        # ----------- TARGET FRAME -----------
        self.correct_frame = "zed2_left_camera_optical_frame"
        # ------------------------------------

        # SUBSCRIBERS
        self.sub_image_raw = self.create_subscription(
            Image,
            "/zed2_left_raw_camera/image_raw",
            self.image_raw_callback,
            10
        )

        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            "/zed2_left_raw_camera/camera_info",
            self.camera_info_callback,
            10
        )

        self.sub_image_rect = self.create_subscription(
            Image,
            "/left/image_rect_color",
            self.image_rect_callback,
            10
        )

        self.sub_camera_info_rect = self.create_subscription(
            CameraInfo,
            "/left/camera_info",
            self.camera_info_rect_callback,
            10
        )

        # PUBLISHERS
        self.pub_image_raw = self.create_publisher(Image, "/zed_fixed/image_raw_color", 10)
        self.pub_camera_info = self.create_publisher(CameraInfo, "/zed_fixed/camera_info", 10)

        self.pub_image_rect = self.create_publisher(Image, "/zed_fixed/image_rect_color", 10)
        self.pub_camera_info_rect = self.create_publisher(CameraInfo, "/zed_fixed/camera_info_rect", 10)

        self.get_logger().info("Camera frame fixer node started.")

    # ------------------ CALLBACKS -----------------------
    def image_raw_callback(self, msg):
        msg.header.frame_id = self.correct_frame
        self.pub_image_raw.publish(msg)

    def camera_info_callback(self, msg):
        msg.header.frame_id = self.correct_frame
        self.pub_camera_info.publish(msg)

    def image_rect_callback(self, msg):
        msg.header.frame_id = self.correct_frame
        self.pub_image_rect.publish(msg)

    def camera_info_rect_callback(self, msg):
        msg.header.frame_id = self.correct_frame
        self.pub_camera_info_rect.publish(msg)
    # ----------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = CameraFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()