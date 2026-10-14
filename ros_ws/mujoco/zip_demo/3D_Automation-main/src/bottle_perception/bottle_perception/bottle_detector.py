import math

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs

from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException, TransformListener
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class BottleDetector(Node):

    def __init__(self):
        super().__init__("bottle_detector")

        self.bridge = CvBridge()
        self.camera_info = None
        self.frame_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/bottle_pose_camera",
            10,
        )

        self.world_pose_publisher = self.create_publisher(
            PoseStamped,
            "/bottle_pose",
            10,
        )

        self.debug_publisher = self.create_publisher(
            Image,
            "/bottle_perception/debug_image",
            10,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/station_camera/station_rgbd/camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.rgb_subscriber = message_filters.Subscriber(
            self,
            Image,
            "/station_camera/station_rgbd/image_raw",
            qos_profile=qos_profile_sensor_data,
        )

        self.depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            "/station_camera/station_rgbd/depth/image_raw",
            qos_profile=qos_profile_sensor_data,
        )

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber],
            queue_size=10,
            slop=0.1,
        )
        self.synchronizer.registerCallback(self.image_callback)

        self.get_logger().info("Bottle detector started")

    def camera_info_callback(self, message):
        self.camera_info = message

    def image_callback(self, rgb_message, depth_message):
        if self.camera_info is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(
                rgb_message,
                desired_encoding="bgr8",
            )
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_message,
                desired_encoding="passthrough",
            )
        except Exception as error:
            self.get_logger().error(f"Image conversion failed: {error}")
            return

        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # 밝고 채도가 높은 파란색 물병 검출
        lower_blue = np.array([90, 140, 160], dtype=np.uint8)
        upper_blue = np.array([130, 255, 255], dtype=np.uint8)

        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        selected_contour = None
        selected_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)

            is_bottle_shape = height > width * 1.3
            is_large_enough = area > 80.0

            if is_bottle_shape and is_large_enough and area > selected_area:
                selected_contour = contour
                selected_area = area

        debug_image = color_image.copy()

        if selected_contour is not None:
            moments = cv2.moments(selected_contour)

            if moments["m00"] > 0.0:
                center_u = int(moments["m10"] / moments["m00"])
                center_v = int(moments["m01"] / moments["m00"])

                contour_mask = np.zeros(mask.shape, dtype=np.uint8)
                cv2.drawContours(
                    contour_mask,
                    [selected_contour],
                    -1,
                    255,
                    thickness=-1,
                )

                valid_depths = depth_image[
                    (contour_mask > 0)
                    & np.isfinite(depth_image)
                    & (depth_image > 0.1)
                    & (depth_image < 5.0)
                ]

                if valid_depths.size > 0:
                    depth_z = float(np.median(valid_depths))

                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    cx = self.camera_info.k[2]
                    cy = self.camera_info.k[5]

                    point_x = (center_u - cx) * depth_z / fx
                    point_y = (center_v - cy) * depth_z / fy
                    point_z = depth_z

                    pose = PoseStamped()
                    pose.header = rgb_message.header
                    pose.header.frame_id = "station_camera_optical_frame"

                    pose.pose.position.x = point_x
                    pose.pose.position.y = point_y
                    pose.pose.position.z = point_z
                    pose.pose.orientation.w = 1.0

                    self.pose_publisher.publish(pose)

                    try:
                        world_pose = self.tf_buffer.transform(
                            pose,
                            "odom",
                            timeout=Duration(seconds=0.1),
                        )
                        self.world_pose_publisher.publish(world_pose)
                    except TransformException as error:
                        self.get_logger().warning(
                            f"TF conversion failed: {error}"
                        )

                    cv2.drawContours(
                        debug_image,
                        [selected_contour],
                        -1,
                        (0, 255, 0),
                        2,
                    )
                    cv2.circle(
                        debug_image,
                        (center_u, center_v),
                        5,
                        (0, 0, 255),
                        -1,
                    )

                    label = (
                        f"x={point_x:.2f} "
                        f"y={point_y:.2f} "
                        f"z={point_z:.2f} m"
                    )

                    cv2.putText(
                        debug_image,
                        label,
                        (center_u + 10, center_v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

                    self.frame_count += 1
                    if self.frame_count % 30 == 0:
                        self.get_logger().info(
                            f"Bottle detected: {label}"
                        )

        debug_message = self.bridge.cv2_to_imgmsg(
            debug_image,
            encoding="bgr8",
        )
        debug_message.header = rgb_message.header
        self.debug_publisher.publish(debug_message)


def main(args=None):
    rclpy.init(args=args)

    node = BottleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()