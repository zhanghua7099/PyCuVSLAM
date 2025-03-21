import os
import sys
import numpy as np
import depthai as dai
import cuvslam as vslam
from typing import Any, Dict
from scipy.spatial.transform import Rotation

# Add the realsense folder to the system path to import visualizer
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../realsense')))

from visualizer import RerunVisualizer

def get_oak_camera(oak_params) -> vslam.Camera:
    """Get a Camera object from a sensor YAML file."""
    cam = vslam.Camera()
    cam.distortion = vslam.Distortion(
            to_distortion_model('polynomial'), oak_params['distortion']
        )
    
    cam.focal = oak_params['intrinsics'][0][0], oak_params['intrinsics'][1][1]
    cam.principal = oak_params['intrinsics'][0][2], oak_params['intrinsics'][1][2]
    cam.size = oak_params['resolution']
    cam.rig_from_camera = oak_transform_to_pose(oak_params['extrinsics'])

    # features within these outer frame will be ignored by cuvslam
    cam.border_top = 100 
    cam.border_bottom = 0
    cam.border_left = 120
    cam.border_right = 120
    return cam

def oak_transform_to_pose(oak_extrinsics) -> vslam.Pose:
    '''Convert 4d transformation matrix to cuvslam pose'''
    return vslam.Pose(rotation = Rotation.from_matrix(np.array(oak_extrinsics)[:3,:3]).as_quat(),
                      translation = np.array(oak_extrinsics)[:3,3]/100) #convert to m

def get_oak_rig(stereo_params) -> vslam.Rig:
    """Get a Rig object from EuRoC dataset path."""
    rig = vslam.Rig()
    rig.cameras = [get_oak_camera(stereo_params['left']), get_oak_camera(stereo_params['right'])]
    return rig

def to_distortion_model(distortion: str) -> vslam.DistortionModel:
    """Convert string distortion model name to vslam.DistortionModel enum."""
    distortion_models = {
        'pinhole': vslam.DistortionModel.Pinhole,
        'fisheye': vslam.DistortionModel.Fisheye,
        'brown': vslam.DistortionModel.Brown,
        'polynomial': vslam.DistortionModel.Polynomial
    }
    
    distortion = distortion.lower()
    if distortion not in distortion_models:
        raise ValueError(f"Unknown distortion model: {distortion}")
        
    return distortion_models[distortion]

def create_oak_pipeline() -> dai.Pipeline:
    """Create and configure the OAK-D pipeline."""
    pipeline = dai.Pipeline()

    # Create stereo pair (left and right cameras)
    left_camera = pipeline.createMonoCamera()
    right_camera = pipeline.createMonoCamera()

    # Set camera properties
    left_camera.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right_camera.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    # Set resolution
    left_camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    right_camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

    # Set FPS
    left_camera.setFps(30)
    right_camera.setFps(30)

    # Create output queues for both cameras
    left_out = pipeline.createXLinkOut()
    right_out = pipeline.createXLinkOut()

    left_out.setStreamName("left")
    right_out.setStreamName("right")

    # Link cameras to output
    left_camera.out.link(left_out.input)
    right_camera.out.link(right_out.input)

    resolution = left_camera.getResolutionSize()
    
    return pipeline, resolution

def get_stereo_calibration(device: dai.Device, resolution) -> Dict[str, Dict[str, Any]]:
    """Get calibration data from the device."""
    stereo_camera = {'left': {}, 'right': {}}
    
    # Set image size
    stereo_camera['left']['resolution'] = resolution
    stereo_camera['right']['resolution'] = resolution

    # Read the calibration data
    calibData = device.readCalibration()

    # Get intrinsics for left and right cameras
    stereo_camera['left']['intrinsics'] = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B)
    stereo_camera['right']['intrinsics'] = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C)

    # Get distortion coefficients for left and right cameras
    stereo_camera['left']['distortion'] = calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)[:8]
    stereo_camera['right']['distortion'] = calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C)[:8]

    # Get extrinsics (relative transformation between cameras)
    stereo_camera['left']['extrinsics'] = calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_A)
    stereo_camera['right']['extrinsics'] = calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_A)
    
    return stereo_camera

prev_timestamp = None
threshold_ns = 35 * 1e6  # 35ms in nanoseconds

visualizer = RerunVisualizer()
frame_id = 0

trajectory = []

# Create and configure the pipeline
pipeline, resolution = create_oak_pipeline()

# Connect to the device and start the pipeline
with dai.Device(pipeline) as device:
    stereo_camera = get_stereo_calibration(device, resolution)
    
    # Output synchronization
    left_queue = device.getOutputQueue("left", 8, False)
    right_queue = device.getOutputQueue("right", 8, False)

    rig = get_oak_rig(stereo_camera)
    cfg = vslam.TrackerConfig(async_sba=False, enable_final_landmarks_export=True, horizontal_stereo_camera=False)
    tracker = vslam.Tracker(rig, cfg)

    # Capture and process stereo frames
    while True:
        left_frame = left_queue.get()
        right_frame = right_queue.get()

        left_ts = left_frame.getTimestamp()
        right_ts = right_frame.getTimestamp()

        timestamp_left = int((left_ts.seconds * 1e9) + (left_ts.microseconds * 1e3))
        timestamp_right = int((right_ts.seconds * 1e9) + (right_ts.microseconds * 1e3))

        # Check if frames are synchronized
        if abs(timestamp_left - timestamp_right) > 5e6:
            print(f"Warning: Stereo pair is not syncronized: timestamp difference: {abs(timestamp_left - timestamp_right)/1e6:.2f} ms")
            continue

            # Check timestamp difference with previous frame
        if prev_timestamp is not None:
            timestamp_diff = timestamp_left - prev_timestamp
            if timestamp_diff > threshold_ns:
                timestamp_diff_ms = timestamp_diff / 1e6  # Convert to milliseconds
                print(f"Warning: Camera stream message drop: timestamp gap ({timestamp_diff/1e6:.2f} ms) exceeds threshold {threshold_ns/1e6:.2f} ms")

        left_img = left_frame.getCvFrame()
        right_img = right_frame.getCvFrame()

        pose_estimate = tracker.track(timestamp_left, (left_img, right_img))
        trajectory.append(pose_estimate.pose.translation)

        frame_id += 1

        visualizer.visualize_frame(
            frame_id=frame_id,
            images=(left_img, right_img),
            pose=pose_estimate.pose,
            observations_main_cam=tracker.get_last_observations(0),
            trajectory=trajectory,
            timestamp=timestamp_left
        )

        prev_timestamp = timestamp_left
