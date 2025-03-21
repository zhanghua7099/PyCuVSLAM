import csv
import os
from typing import List

import numpy as np
import yaml
from PIL import Image
from scipy.spatial.transform import Rotation

import cuvslam as vslam


def load_frame(image_path: str) -> np.ndarray:
    """Load an image from a file path."""
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Image file not found: {image_path}")

    image = Image.open(image_path)
    frame = np.array(image)

    if image.mode == 'L':
        # mono8
        if len(frame.shape) != 2:
            raise ValueError("Expected mono8 image to have 2 dimensions [H W].")
    elif image.mode == 'RGB':
        # rgb8
        if len(frame.shape) != 3 or frame.shape[2] != 3:
            raise ValueError(
                "Expected rgb8 image to have 3 dimensions with 3 channels [H W C].")
    else:
        raise ValueError(f"Unsupported image mode: {image.mode}")

    return frame


def transform_to_pose(transform_16: List[float]) -> vslam.Pose:
    """Convert a 4x4 transformation matrix to a vslam.Pose object."""
    transform = np.array(transform_16).reshape(4, 4)
    rotation_quat = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return vslam.Pose(rotation=rotation_quat, translation=transform[:3, 3])


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


def get_camera(yaml_path: str) -> vslam.Camera:
    """Get a Camera object from a sensor YAML file."""
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Sensor YAML not found: {yaml_path}")
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    cam = vslam.Camera()
    cam.distortion = vslam.Distortion(
        to_distortion_model(config['camera_model']),
        config['distortion_coefficients']
    )
    cam.focal = config['intrinsics'][0:2]
    cam.principal = config['intrinsics'][2:4]
    cam.size = config['resolution']
    cam.rig_from_camera = transform_to_pose(config['T_Cam0']['data'])
    return cam


def get_imu_calibration(yaml_path: str) -> vslam.ImuCalibration:
    """Get an ImuCalibration object from a sensor YAML file."""
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Sensor YAML not found: {yaml_path}")
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    imu = vslam.ImuCalibration()
    imu.rig_from_imu = transform_to_pose(config['T_Cam0']['data'])
    imu.gyroscope_noise_density = config['gyroscope_noise_density']
    imu.gyroscope_random_walk = config['gyroscope_random_walk']
    imu.accelerometer_noise_density = config['accelerometer_noise_density']
    imu.accelerometer_random_walk = config['accelerometer_random_walk']
    imu.frequency = config['rate_hz']
    return imu


def get_rig(euroc_path: str) -> vslam.Rig:
    """Get a Rig object from EuRoC dataset path."""
    if not os.path.exists(euroc_path):
        raise FileNotFoundError(f"EuRoC dataset path does not exist: {euroc_path}")
    rig = vslam.Rig()
    rig.cameras = [get_camera(os.path.join(euroc_path, cam, 'sensor_recalibrated.yaml'))
                   for cam in ['cam0', 'cam1']]
    rig.imus = [get_imu_calibration(os.path.join(euroc_path, 'imu0', 'sensor_recalibrated.yaml'))]
    return rig


def read_csv_data(csv_path: str, sensor_type: str) -> List[dict]:
    """Read timestamp and sensor data from CSV file."""
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    valid_sensors = {'camera', 'imu'}
    if sensor_type not in valid_sensors:
        raise ValueError(f"Unknown sensor type: {sensor_type}. Must be one of {valid_sensors}")

    data = []
    with open(csv_path, 'r') as f:
        next(f)  # Skip header line
        reader = csv.reader(f)
        for row in reader:
            if not row:  # Skip empty rows
                continue
            if sensor_type == 'camera':
                if len(row) < 2:
                    raise ValueError(f"Invalid camera data format. Expected timestamp and filename, got: {row}")
                data.append({
                    'timestamp': int(row[0]),
                    'filename': row[1]
                })
            else:  # imu
                if len(row) < 7:
                    raise ValueError(f"Invalid IMU data format. Expected timestamp and 6 values, got: {row}")
                data.append({
                    'timestamp': int(row[0]),
                    'gyro': [float(x) for x in row[1:4]],
                    'accel': [float(x) for x in row[4:7]]
                })

    return data


def prepare_frame_metadata_euroc(euroc_path: str, odometry_mode: vslam.TrackerOdometryMode) -> List[dict]:
    """Process EuRoC dataset camera files and generate synchronized frame metadata."""
    if not os.path.exists(euroc_path):
        raise ValueError(f"EuRoC dataset path does not exist: {euroc_path}")

    left_cam_csv = os.path.join(euroc_path, 'cam0', 'data.csv')
    right_cam_csv = os.path.join(euroc_path, 'cam1', 'data.csv')

    if not os.path.exists(left_cam_csv) or not os.path.exists(right_cam_csv):
        raise ValueError(f"Camera data CSV files not found in {left_cam_csv} or {right_cam_csv}")

    left_cam_data = read_csv_data(left_cam_csv, 'camera')
    right_cam_data = read_csv_data(right_cam_csv, 'camera')

    # Verify timestamps match between cameras
    if len(left_cam_data) != len(right_cam_data):
        raise ValueError("Number of frames differs between left and right cameras")

    for i, (left_cam, right_cam) in enumerate(zip(left_cam_data, right_cam_data)):
        if left_cam['timestamp'] != right_cam['timestamp']:
            raise ValueError(f"Timestamp mismatch at frame {i}")

    # Generate stereo frame metadata
    frames_metadata = [{
        'type': 'stereo',
        'timestamp': left_cam['timestamp'],
        'images_paths': [
            os.path.join(euroc_path, 'cam0', 'data', left_cam['filename']),
            os.path.join(euroc_path, 'cam1', 'data', right_cam['filename']),
        ],
    } for left_cam, right_cam in zip(left_cam_data, right_cam_data)]

    # Add IMU data if in inertial mode
    if odometry_mode == vslam.TrackerOdometryMode.Inertial:
        imu_csv = os.path.join(euroc_path, 'imu0', 'data.csv')
        if not os.path.exists(imu_csv):
            raise ValueError(f"IMU data CSV file not found in {imu_csv}")

        imu_data = read_csv_data(imu_csv, 'imu')
        frames_metadata.extend({
                                   'type': 'imu',
                                   'timestamp': measurement['timestamp'],
                                   'gyro': measurement['gyro'],
                                   'accel': measurement['accel']
                               } for measurement in imu_data)

    # Sort frames by timestamp
    frames_metadata.sort(key=lambda x: x['timestamp'])

    return frames_metadata
