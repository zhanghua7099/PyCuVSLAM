#
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
from typing import List, Optional

import numpy as np
import cv2
import time

from pyorbbecsdk import *
from typing import Union, Any, Optional

import cuvslam as vslam
from visualizer import RerunVisualizer

# Constants
WARMUP_FRAMES = 60
IMAGE_JITTER_THRESHOLD_MS = 50 * 1e6  # 50ms in nanoseconds
NUM_VIZ_CAMERAS = 2
VIZ_MAP_POINTS = True

def frame_to_bgr_image(frame: VideoFrame) -> Union[Optional[np.array], Any]:
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())
    image = np.zeros((height, width, 3), dtype=np.uint8)
    if color_format == OBFormat.RGB:
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif color_format == OBFormat.BGR:
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif color_format == OBFormat.YUYV:
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
    elif color_format == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    elif color_format == OBFormat.I420:
        image = i420_to_bgr(data, width, height)
        return image
    elif color_format == OBFormat.NV12:
        image = nv12_to_bgr(data, width, height)
        return image
    elif color_format == OBFormat.NV21:
        image = nv21_to_bgr(data, width, height)
        return image
    elif color_format == OBFormat.UYVY:
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
    else:
        print("Unsupported color format: {}".format(color_format))
        return None
    return image


def main() -> None:
    """Main function for RGBD tracking."""
    # Configure depth and color streams
    pipeline = Pipeline()
    config = Config()
    device = pipeline.get_device()
    device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, True)
    device.set_bool_property(OBPropertyID.OB_PROP_LASER_BOOL, True)

    enable_sync = True
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(color_profile)
        profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        assert profile_list is not None
        depth_profile = profile_list.get_default_video_stream_profile()
        assert depth_profile is not None
        print("color profile : {}x{}@{}_{}".format(color_profile.get_width(),
                                                   color_profile.get_height(),
                                                   color_profile.get_fps(),
                                                   color_profile.get_format()))
        print("depth profile : {}x{}@{}_{}".format(depth_profile.get_width(),
                                                   depth_profile.get_height(),
                                                   depth_profile.get_fps(),
                                                   depth_profile.get_format()))
        config.enable_stream(depth_profile)
    except Exception as e:
        print(e)

    # use software/hardware alignment
    config.set_align_mode(OBAlignMode.SW_MODE)
    # config.set_align_mode(OBAlignMode.HW_MODE)
    
    if enable_sync:
        try:
            pipeline.enable_frame_sync()
        except Exception as e:
            print(e)
    try:
        pipeline.start(config)
    except Exception as e:
        print(e)

    # get default calibration parameters
    calib_camera_params_list = device.get_calibration_camera_param_list()
    calib_camera_params = calib_camera_params_list.get_camera_param(0)
    fx = calib_camera_params.rgb_intrinsic.fx
    fy = calib_camera_params.rgb_intrinsic.fy
    cx = calib_camera_params.rgb_intrinsic.cx
    cy = calib_camera_params.rgb_intrinsic.cy
    k1 = calib_camera_params.rgb_distortion.k1
    k2 = calib_camera_params.rgb_distortion.k2
    p1 = calib_camera_params.rgb_distortion.p1
    p2 = calib_camera_params.rgb_distortion.p2
    k3 = calib_camera_params.rgb_distortion.k3
    
    # Configure RGBD settings
    rgbd_settings = vslam.Tracker.OdometryRGBDSettings()
    rgbd_settings.depth_scale_factor = 1000.0
    rgbd_settings.depth_camera_id = 0
    rgbd_settings.enable_depth_stereo_tracking = False

    # Configure tracker
    cfg = vslam.Tracker.OdometryConfig(
        async_sba=True,
        enable_final_landmarks_export=True,
        odometry_mode=vslam.Tracker.OdometryMode.RGBD,
        rgbd_settings=rgbd_settings
    )

    # Create rig using utility function
    rig = vslam.Rig()
    cam = vslam.Camera()
    cam.distortion = vslam.Distortion(
        vslam.Distortion.Model.Brown,
        [k1, k2, k3, p1, p2]
    )
    cam.focal = fx, fy
    cam.principal = cx, cy
    cam.size = 640, 480
    rig.cameras = [cam]

    # Initialize tracker and visualizer
    tracker = vslam.Tracker(rig, cfg)

    visualizer = RerunVisualizer(num_viz_cameras=NUM_VIZ_CAMERAS)

    frame_id = 0
    prev_timestamp: Optional[int] = None
    trajectory: List[np.ndarray] = []

    try:
        while True:
            # Wait for frames
            frames: FrameSet = pipeline.wait_for_frames(1)
            if frames is None:
                    continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            color_image = frame_to_bgr_image(color_frame)
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue

            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_data = depth_data.reshape((480, 640))

            timestamp = int(depth_frame.get_timestamp() * 1e6)  # Convert to nanoseconds

            # Check timestamp difference with previous frame
            if prev_timestamp is not None:
                timestamp_diff = timestamp - prev_timestamp
                if timestamp_diff > IMAGE_JITTER_THRESHOLD_MS:
                    print(
                        f"Warning: Camera stream message drop: timestamp gap "
                        f"({timestamp_diff/1e6:.2f} ms) exceeds threshold "
                        f"{IMAGE_JITTER_THRESHOLD_MS/1e6:.2f} ms"
                    )

            frame_id += 1

            # Warmup for specified number of frames
            if frame_id > WARMUP_FRAMES:
                images = [
                    color_image,
                    depth_data
                ]

                # Track frame
                odom_pose_estimate, _ = tracker.track(
                    timestamp, images=[images[0]], depths=[images[1]]
                )

                odom_pose = odom_pose_estimate.world_from_rig.pose
                trajectory.append(odom_pose.translation)

                # Store current timestamp for next iteration
                prev_timestamp = timestamp

                # Visualize results for color and depth cameras
                # Same observations for both, since we only have one image
                observations = tracker.get_last_observations(0)

                # Get final landmarks for visualization
                landmarks = list(tracker.get_final_landmarks().values()) if VIZ_MAP_POINTS else None
                visualizer.visualize_frame(
                    frame_id=frame_id,
                    images=images,
                    pose=odom_pose,
                    observations_main_cam=[observations, observations],
                    trajectory=trajectory,
                    final_landmarks=landmarks,
                    timestamp=timestamp
                )

    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
