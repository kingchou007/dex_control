import os
import cv2
import numpy as np
import time
import threading
import click
import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster

class TargetingNode(Node):
    """
    ROS 2 node for eye-to-hand calibration using a Realsense camera and ArUco markers.
    
    This class handles the end-to-end calibration pipeline. It subscribes to RGB image 
    streams, camera intrinsics, and the robot's end-effector pose. Using the Tsai-Lenz 
    method, it computes the transformation matrix between the robot's base frame and 
    the camera's optical frame. During the calibration loop, it records raw RGB images, 
    annotated visualization frames, and End-Effector trajectories into a timestamped 
    directory for dataset generation and reproducibility. Finally, it broadcasts the 
    computed transformation to the ROS 2 TF tree and saves final matrices to disk.
    """
    def __init__(self, test_views=20):
        super().__init__('eye2hand_calib')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.rgb_sub = self.create_subscription(
            Image, 
            "/camera/camera/color/image_raw", 
            self._bgr_callback,
            qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._camera_info_callback,
            qos_profile_sensor_data
        )

        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            "/franka/ee_pose",
            self._ee_pose_callback,
            10
        )

        self.aruco_rgb_pub = self.create_publisher(Image, "/aruco_rgb", 10)
        self.target_pose_pub = self.create_publisher(Pose, "/target_pose", 10)

        self.camera_info_loaded = False
        self._cv_bridge = CvBridge()
        # Adjust according to your ArUco marker size (in meters)
        self.marker_size = 0.094        
        self.aruco_id = 582       
        self.test_views = test_views
        
        self.origin_image = None
        self.bgr_image = None
        self.intrinsic_matrix_dict = None
        
        self.g2r = None
        self.current_t = None
        self.current_q = None
        self.trans_mats = []
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(".", "calibration_results", timestamp)
        self.rgb_dir = os.path.join(self.save_dir, "rgb")
        self.vis_dir = os.path.join(self.save_dir, "vis")
        self.per_frame_dir = os.path.join(self.save_dir, "per_frame_results")
        
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.vis_dir, exist_ok=True)
        os.makedirs(self.per_frame_dir, exist_ok=True)
        
        self.csv_path = os.path.join(self.save_dir, "trajectory.csv")
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["idx", "PX", "PY", "PZ", "QX", "QY", "QZ", "QW"])
        
        self.latest_c2r = None

    def _ee_pose_callback(self, msg: PoseStamped):
        try:
            self.current_t = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.current_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

            rot_matrix = R.from_quat(self.current_q).as_matrix()
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = rot_matrix
            transformation_matrix[:3, 3] = self.current_t
            
            self.g2r = transformation_matrix
        except Exception as e:
            self.get_logger().error(f"Failed to process pose from topic: {e}")

    def _camera_info_callback(self, msg):
        if not self.camera_info_loaded:
            self.intrinsic_matrix_dict = {
                "fx": msg.k[0],
                "fy": msg.k[4],
                "cx": msg.k[2],
                "cy": msg.k[5],
            }
            self.camera_info_loaded = True

    def _bgr_callback(self, msg):
        if not self.camera_info_loaded:
            return

        self.origin_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.bgr_image = self.origin_image.copy()

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        
        try:
            aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            aruco_params = cv2.aruco.DetectorParameters_create()

        detection_result = cv2.aruco.detectMarkers(self.bgr_image, aruco_dict, parameters=aruco_params)
        corners, ids = detection_result[0], detection_result[1]

        mtx = np.array([
            [self.intrinsic_matrix_dict["fx"], 0, self.intrinsic_matrix_dict["cx"]],
            [0, self.intrinsic_matrix_dict["fy"], self.intrinsic_matrix_dict["cy"]],
            [0, 0, 1],
        ], dtype=np.float64) 

        dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64) 

        if ids is not None and len(corners) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, mtx, dist) 

            self.trans_mats = []
            filter_corners = []
            filter_ids = []
            
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.aruco_id:
                    rvec, tvec = rvecs[i], tvecs[i]

                    R_mat, _ = cv2.Rodrigues(rvec[0] if rvec.ndim > 1 else rvec)

                    trans_mat = np.eye(4)
                    trans_mat[:3, :3] = R_mat
                    trans_mat[:3, 3] = tvec.flatten()
                    
                    cv2.drawFrameAxes(self.bgr_image, mtx, dist, rvec, tvec, 0.05)

                    self.trans_mats.append(trans_mat)
                    filter_corners.append(corners[i])
                    filter_ids.append(marker_id)

            image_markers = cv2.aruco.drawDetectedMarkers(
                self.bgr_image.copy(), filter_corners, np.array(filter_ids)
            )
            self.aruco_rgb_pub.publish(self._cv_bridge.cv2_to_imgmsg(image_markers, encoding="bgr8"))
        else:
            self.trans_mats = []
            self.aruco_rgb_pub.publish(self._cv_bridge.cv2_to_imgmsg(self.bgr_image, encoding="bgr8"))

    def vis_targeting(self, current_view_idx):
        if self.latest_c2r is not None:
            try:
                T_camera_to_base = self.latest_c2r
                T_base_to_camera = np.linalg.inv(T_camera_to_base)

                axis_length = 0.1
                axes_points_base = np.array([
                    [0, 0, 0],           
                    [axis_length, 0, 0], 
                    [0, axis_length, 0], 
                    [0, 0, axis_length]  
                ])
   
                ones = np.ones((axes_points_base.shape[0], 1))
                axes_points_base_homogeneous = np.hstack([axes_points_base, ones])

                axes_points_camera = (T_base_to_camera @ axes_points_base_homogeneous.T).T
                points_3D = axes_points_camera[:, :3]

                mtx = np.array([
                    [self.intrinsic_matrix_dict["fx"], 0, self.intrinsic_matrix_dict["cx"]],
                    [0, self.intrinsic_matrix_dict["fy"], self.intrinsic_matrix_dict["cy"]],
                    [0, 0, 1]
                ], dtype=np.float64)
                dist = np.zeros(5, dtype=np.float64)
                rvec = np.zeros((3, 1))
                tvec = np.zeros((3, 1))

                projected_points, _ = cv2.projectPoints(points_3D, rvec, tvec, mtx, dist)
                projected_points = projected_points.reshape(-1, 2)

                origin = tuple(projected_points[0].astype(int))
                x_axis = tuple(projected_points[1].astype(int))
                y_axis = tuple(projected_points[2].astype(int))
                z_axis = tuple(projected_points[3].astype(int))

                if self.origin_image is not None:
                    image = self.origin_image.copy()
                    cv2.line(image, origin, x_axis, (0, 0, 255), 2)    
                    cv2.line(image, origin, y_axis, (0, 255, 0), 2)     
                    cv2.line(image, origin, z_axis, (255, 0, 0), 2)    
                    cv2.circle(image, origin, radius=5, color=(0, 0, 0), thickness=-1)

                    img_filename = os.path.join(self.vis_dir, f"vis_view_{current_view_idx}.jpg")
                    cv2.imwrite(img_filename, image)

                    cv2.imshow("Base Position and Orientation", image)
                    cv2.waitKey(2000) 
                    cv2.destroyAllWindows()
            except Exception as e:
                self.get_logger().error(f"Visualization error: {e}")

        if not self.trans_mats:
            return None

        return self.trans_mats[0]

    def broadcast_tf(self, c2r):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'panda_link0' 
        t.child_frame_id = 'camera_color_optical_frame'
        
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = c2r[:3, 3]
        quat = R.from_matrix(c2r[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
        
        self.tf_static_broadcaster.sendTransform(t)

    def calibrate(self):
        o2cs = []  
        g2rs = []  

        print("\nStarting calibration loop...")

        while rclpy.ok():
            current_idx = len(o2cs)
            flag = input(f"\n>> Press 'q' to end calibration, or ENTER to capture view {current_idx}: ").lower()
            if flag == "q":
                break

            if not self.camera_info_loaded or self.origin_image is None:
                print("Waiting for camera topics...")
                continue

            o2c = self.vis_targeting(current_idx)
            
            if o2c is not None:
                if self.g2r is not None and self.current_t is not None and self.current_q is not None:
                    o2cs.append(o2c)
                    g2rs.append(np.linalg.inv(self.g2r))
                    
                    rgb_filename = os.path.join(self.rgb_dir, f"{current_idx}.jpg")
                    cv2.imwrite(rgb_filename, self.origin_image)
                    
                    with open(self.csv_path, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            current_idx,
                            self.current_t[0], self.current_t[1], self.current_t[2],
                            self.current_q[0], self.current_q[1], self.current_q[2], self.current_q[3]
                        ])
                        
                    print(f"Calibration data collected. {len(o2cs)} views.")
                else:
                    print("Waiting for Robot EE Pose from /franka/ee_pose topic...")
            else:
                print(f"No AruCo marker ID {self.aruco_id} detected in current frame.")

            if len(o2cs) >= 3:
                R_gripper2base = [g[:3, :3] for g in g2rs]
                t_gripper2base = [g[:3, 3] for g in g2rs]
                R_obj2cam = [o[:3, :3] for o in o2cs]
                t_obj2cam = [o[:3, 3] for o in o2cs]

                R_cam2base, t_cam2base = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base, 
                    R_obj2cam, t_obj2cam, 
                    method=cv2.CALIB_HAND_EYE_TSAI
                )

                c2r = np.eye(4)
                c2r[:3, :3] = R_cam2base
                c2r[:3, 3] = t_cam2base[:, 0]
                
                self.latest_c2r = c2r
                print(f"\nCurrent Calibration ({len(o2cs)} views):\n{c2r}")
                
                np.save(os.path.join(self.per_frame_dir, f"{len(o2cs)}views_c2r.npy"), c2r)
                np.savetxt(os.path.join(self.per_frame_dir, f"{len(o2cs)}views_c2r.csv"), c2r, delimiter=",")
                
                self.broadcast_tf(c2r)

        if len(o2cs) > 0:
            np.save(os.path.join(self.save_dir, 'o2cs.npy'), np.stack(o2cs[:20]))
            np.save(os.path.join(self.save_dir, 'g2rs.npy'), np.stack(g2rs[:20]))
            if self.latest_c2r is not None:
                np.save(os.path.join(self.save_dir, 'c2r.npy'), self.latest_c2r)
                np.savetxt(os.path.join(self.save_dir, 'c2r.csv'), self.latest_c2r, delimiter=",")
            print(f"Saved final matrices to {self.save_dir}")

@click.command()
@click.option("-t", "--test_views", default=20)
def main(test_views):
    rclpy.init()
    node = TargetingNode(test_views=test_views)
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(2)

    try:
        node.calibrate()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
