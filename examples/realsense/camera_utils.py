import numpy as np
import cuvslam as vslam
from scipy.spatial.transform import Rotation
from typing import Dict, Any, Tuple

def to_odometry_mode(mode: str) -> vslam.TrackerOdometryMode:
    """Convert string odometry mode to vslam.TrackerOdometryMode enum."""
    odometry_modes = {
        'mono': vslam.TrackerOdometryMode.Mono,
        'stereo': vslam.TrackerOdometryMode.Multicamera,
        'inertial': vslam.TrackerOdometryMode.Inertial
    }
    
    mode = mode.lower()
    if mode not in odometry_modes:
        raise ValueError(f"Unknown odometry mode: {mode}")
        
    return odometry_modes[mode]

def transform_to_pose(rs_transform = None) -> vslam.Pose:
    if rs_transform:
        """Convert transformations provided by realsense to a vslam.Pose object."""
        rotation_matrix, translation_vec  = np.array(rs_transform.rotation).reshape([3, 3]), rs_transform.translation
        rotation_quat = Rotation.from_matrix(rotation_matrix).as_quat()
    else:
        rotation_matrix, translation_vec = np.eye(3), [0]*3
        rotation_quat = Rotation.from_matrix(rotation_matrix).as_quat()
    return vslam.Pose(rotation=rotation_quat, translation=translation_vec)

# IMU is in opengl coordinate system
def rig_from_imu_pose(rs_transform = None) -> vslam.Pose:
    rotation_matrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    rotation_quat = Rotation.from_matrix(rotation_matrix).as_quat()
    translation_vec = rotation_matrix @ rs_transform.translation
    return vslam.Pose(rotation=rotation_quat, translation=translation_vec)

    
def get_rs_camera(rs_intrinsics, rs_extinsincs = None) -> vslam.Camera:
    """Get a Camera object from RealSense intrinsics."""
    cam = vslam.Camera()
    cam.distortion = vslam.Distortion(vslam.DistortionModel.Pinhole)
    cam.focal = rs_intrinsics.fx, rs_intrinsics.fy
    cam.principal = rs_intrinsics.ppx, rs_intrinsics.ppy
    cam.size = rs_intrinsics.width, rs_intrinsics.height
    cam.rig_from_camera = transform_to_pose(rs_extinsincs)
    return cam

def get_rs_imu(imu_extrinsics, frequency: int = 200) -> vslam.ImuCalibration:
    """Get an IMU calibration object from RealSense extrinsics."""
    imu = vslam.ImuCalibration()
    imu.rig_from_imu = rig_from_imu_pose(imu_extrinsics)
    imu.gyroscope_noise_density = 6.0673370376614875e-03
    imu.gyroscope_random_walk = 3.6211951458325785e-05
    imu.accelerometer_noise_density = 3.3621979208052800e-02
    imu.accelerometer_random_walk = 9.8256589971851467e-04
    imu.frequency = frequency
    return imu

def get_rs_stereo_rig(camera_params: Dict[str, Dict[str, Any]]) -> vslam.Rig:
    """Get a VIO Rig object with cameras from RealSense parameters."""
    rig = vslam.Rig()
    rig.cameras = [
        get_rs_camera(camera_params['left']['intrinsics']), 
        get_rs_camera(camera_params['right']['intrinsics'], camera_params['right']['extrinsics'])
    ]
    return rig 

def get_rs_vio_rig(camera_params: Dict[str, Dict[str, Any]]) -> vslam.Rig:
    """Get a VIO Rig object with cameras and IMU from RealSense parameters."""
    rig = vslam.Rig()
    rig.cameras = [
        get_rs_camera(camera_params['left']['intrinsics']), 
        get_rs_camera(camera_params['right']['intrinsics'], camera_params['right']['extrinsics'])
    ]
    rig.imus = [get_rs_imu(camera_params['imu']['cam_from_imu'])]
    return rig 