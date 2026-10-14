#!/usr/bin/env python3

import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class Tron1MuJoCoVision(Node):

    MODEL_PATH = "/ros_ws/mujoco/scenes/warehouse_tron1_final_visual.xml"
    CAMERA_NAME = "station_rgbd_camera"
    BOTTLE_GEOM_NAME = "carry_bottle_collision"

    WIDTH = 640
    HEIGHT = 480

    def __init__(self):
        super().__init__("tron1_mujoco_vision")

        self.model = mujoco.MjModel.from_xml_path(self.MODEL_PATH)
        self.data = mujoco.MjData(self.model)

        self.camera_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            self.CAMERA_NAME,
        )

        self.bottle_geom_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            self.BOTTLE_GEOM_NAME,
        )

        if self.camera_id < 0:
            raise RuntimeError(f"Camera not found: {self.CAMERA_NAME}")

        if self.bottle_geom_id < 0:
            raise RuntimeError(
                f"Bottle geom not found: {self.BOTTLE_GEOM_NAME}"
            )

        mujoco.mj_forward(self.model, self.data)

        # Runtime-only RGB-D camera look-at correction.
        # The station camera is attached to station_camera_stand,
        # so the quaternion must be expressed in the parent-body frame.
        from scipy.spatial.transform import Rotation

        cam_body_id = self.model.cam_bodyid[self.camera_id]
        cam_pos = self.data.xpos[cam_body_id].copy()
        target = self.data.geom_xpos[self.bottle_geom_id].copy()

        forward = target - cam_pos
        forward /= np.linalg.norm(forward)

        z_axis = -forward
        up = np.array([0.0, 0.0, 1.0])
        x_axis = np.cross(up, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        Rwc = np.column_stack((x_axis, y_axis, z_axis))
        Rwb = self.data.xmat[cam_body_id].reshape(3, 3)
        Rbc = Rwb.T @ Rwc

        q_xyzw = Rotation.from_matrix(Rbc).as_quat()
        self.model.cam_quat[self.camera_id] = q_xyzw[[3, 0, 1, 2]]

        mujoco.mj_forward(self.model, self.data)

        self.renderer = mujoco.Renderer(
            self.model,
            height=self.HEIGHT,
            width=self.WIDTH,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/detected_bottle_pose",
            10,
        )

        self.stopped_pub = self.create_publisher(
            Bool,
            "/detected_bottle_stopped",
            10,
        )

        self.last_position = None
        self.stable_count = 0

        self.timer = self.create_timer(
            0.1,
            self.process_frame,
        )

        self.get_logger().info(
            "TRON1 MuJoCo Vision initialized"
        )
        self.get_logger().info(
            f"camera={self.CAMERA_NAME} "
            f"id={self.camera_id}"
        )
        self.get_logger().info(
            f"bottle_geom={self.BOTTLE_GEOM_NAME} "
            f"id={self.bottle_geom_id}"
        )

    def process_frame(self):
        mujoco.mj_forward(self.model, self.data)

        self.renderer.update_scene(
            self.data,
            camera=self.camera_id,
        )

        # MuJoCo 3.6 segmentation rendering.
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = 1
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = 1

        seg = self.renderer.render()

        if seg.ndim != 3 or seg.shape[2] < 2:
            self.get_logger().warning(
                "Unexpected segmentation buffer"
            )
            return

        seg0 = seg[:, :, 0].astype(np.int32)

        # MuJoCo 3.6 ID-color segmentation:
        # geom ID is encoded in the first channel.
        bottle_mask = (seg0 == self.bottle_geom_id)

        ys, xs = np.where(bottle_mask)

        if len(xs) == 0:
            self.get_logger().warning(
                "Bottle not visible in segmentation"
            )
            return

        # Actual depth buffer.
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render()
        self.renderer.disable_depth_rendering()

        valid = (
            bottle_mask &
            np.isfinite(depth) &
            (depth > 0.0)
        )

        ys, xs = np.where(valid)

        if len(xs) == 0:
            self.get_logger().warning(
                "No valid bottle depth"
            )
            return

        # Camera intrinsics for 640x480, fovy=60 deg.
        fovy = np.deg2rad(
            self.model.cam_fovy[self.camera_id]
        )

        fy = (self.HEIGHT / 2.0) / np.tan(fovy / 2.0)
        fx = fy
        cx = (self.WIDTH - 1) / 2.0
        cy = (self.HEIGHT - 1) / 2.0

        z = depth[ys, xs]

        # MuJoCo camera convention:
        # local -Z = optical axis
        x_cam = (xs - cx) * z / fx
        y_cam = -(ys - cy) * z / fy
        z_cam = -z

        points_cam = np.column_stack(
            (x_cam, y_cam, z_cam)
        )

        # Camera pose in world coordinates.
        cam_pos = self.data.cam_xpos[self.camera_id].copy()
        cam_mat = self.data.cam_xmat[self.camera_id].reshape(3, 3)

        points_world = (
            points_cam @ cam_mat.T
        ) + cam_pos

        # Recover physical bottle center from visible top surface.
        p_min = points_world.min(axis=0)
        p_max = points_world.max(axis=0)

        detected = np.array([
            (p_min[0] + p_max[0]) / 2.0,
            (p_min[1] + p_max[1]) / 2.0,
            p_max[2] - 0.100,
        ])

        # Stability check.
        stopped = Bool()

        if self.last_position is None:
            self.stable_count = 0
        else:
            delta = np.linalg.norm(
                detected - self.last_position
            )

            if delta < 0.002:
                self.stable_count += 1
            else:
                self.stable_count = 0

        self.last_position = detected.copy()

        stopped.data = self.stable_count >= 3

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = float(detected[0])
        msg.pose.position.y = float(detected[1])
        msg.pose.position.z = float(detected[2])

        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)
        self.stopped_pub.publish(stopped)

        self.get_logger().info(
            "VISION bottle pose: "
            f"[{detected[0]:.6f}, "
            f"{detected[1]:.6f}, "
            f"{detected[2]:.6f}] "
            f"stable={stopped.data}",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)

    node = Tron1MuJoCoVision()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
