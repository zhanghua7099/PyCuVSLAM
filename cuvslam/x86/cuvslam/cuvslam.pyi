from typing import Dict, List, Optional, Tuple, Union, Sequence, overload
import numpy as np
import numpy.typing as npt


# Enum definitions

class DistortionModel:
    Pinhole: int
    Brown: int
    Fisheye: int
    Polynomial: int


class TrackerMulticameraMode:
    Performance: int
    Precision: int
    Moderate: int


class TrackerOdometryMode:
    Multicamera: int
    Inertial: int
    Mono: int


# Main classes

class Pose:
    rotation: Union[List[float], npt.NDArray[np.float32]]  # shape (4,)
    translation: Union[List[float], npt.NDArray[np.float32]]  # shape (3,)

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, rotation: Union[Sequence[float], npt.NDArray[np.float32]],
                 translation: Union[Sequence[float], npt.NDArray[np.float32]]) -> None: ...

    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


class Distortion:
    model: Union[DistortionModel, int]
    parameters: Sequence[float]

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, model: Union[DistortionModel, int],
                 parameters: Sequence[float] = ...) -> None: ...

    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


class Camera:
    size: Union[List[int], npt.NDArray[np.int32]]  # shape (2,)
    principal: Union[List[float], npt.NDArray[np.float32]]  # shape (2,)
    focal: Union[List[float], npt.NDArray[np.float32]]  # shape (2,)
    rig_from_camera: Pose
    distortion: Distortion
    border_top: int
    border_bottom: int
    border_left: int
    border_right: int

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(
        self,
        *,
        size: Union[Sequence[int], npt.NDArray[np.int32]],
        principal: Union[Sequence[float], npt.NDArray[np.float32]],
        focal: Union[Sequence[float], npt.NDArray[np.float32]],
        rig_from_camera: Pose = ...,
        distortion: Distortion = ...,
        border_top: int = 0,
        border_bottom: int = 0,
        border_left: int = 0,
        border_right: int = 0
    ) -> None: ...
    def __str__(self) -> str: ...
    def __repr__(self) -> str: ...


class ImuCalibration:
    rig_from_imu: Pose
    gyroscope_noise_density: float
    accelerometer_noise_density: float
    gyroscope_random_walk: float
    accelerometer_random_walk: float
    frequency: float

    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...


class Rig:
    cameras: List[Camera]
    imus: List[ImuCalibration]

    def __init__(
        self,
        cameras: List[Camera] = ...,
        imus: List[ImuCalibration] = ...
    ) -> None: ...
    def __repr__(self) -> str: ...


class PoseEstimate:
    timestamp_ns: int
    is_valid: bool
    pose: Pose

    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...


class Observation:
    id: int
    u: float
    v: float
    camera_index: int

    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...


class Landmark:
    id: int
    coords: Union[Sequence[float], List[float]]  # length 3, x,y,z coordinates

    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...


class ImuMeasurement:
    timestamp_ns: int
    linear_accelerations: Union[Sequence[float], npt.NDArray[np.float32]]
    angular_velocities: Union[Sequence[float], npt.NDArray[np.float32]]

    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...


# Tracker related classes

class TrackerConfig:
    multicam_mode: Union[TrackerMulticameraMode, int]
    odometry_mode: Union[TrackerOdometryMode, int]
    use_gpu: bool
    async_sba: bool
    use_motion_model: bool
    use_denoising: bool
    horizontal_stereo_camera: bool
    enable_observations_export: bool
    enable_landmarks_export: bool
    enable_final_landmarks_export: bool
    max_frame_delta_s: float
    debug_dump_directory: str
    debug_imu_mode: bool
    def __init__(
        self,
        *,
        multicam_mode: Union[TrackerMulticameraMode, int] = ...,
        odometry_mode: Union[TrackerOdometryMode, int] = ...,
        use_gpu: bool = ...,
        async_sba: bool = ...,
        use_motion_model: bool = ...,
        use_denoising: bool = ...,
        horizontal_stereo_camera: bool = ...,
        enable_observations_export: bool = ...,
        enable_landmarks_export: bool = ...,
        enable_final_landmarks_export: bool = ...,
        max_frame_delta_s: float = ...,
        debug_dump_directory: str = ...,
        debug_imu_mode: bool = ...
    ) -> None: ...


class Tracker:
    def __init__(self, rig: Rig, cfg: TrackerConfig = ...) -> None: ...

    def track(
        self,
        timestamp: int,
        images: List[npt.NDArray[np.uint8]],
        masks: Optional[List[npt.NDArray[np.uint8]]] = None
    ) -> PoseEstimate: ...

    def register_imu_measurement(
        self,
        sensor_index: int,
        imu_measurement: ImuMeasurement
    ) -> None: ...

    def get_last_observations(
        self,
        camera_index: int
    ) -> List[Observation]: ...
    def get_last_landmarks(self) -> List[Landmark]: ...
    def get_last_gravity(self) -> Optional[npt.NDArray[np.float32]]: ...
    def get_final_landmarks(self) -> Dict[int, npt.NDArray[np.float32]]: ...


# Free functions

def get_version() -> Tuple[bool, int, int]: ...
def set_verbosity(verbosity: int) -> None: ...
def warm_up_gpu() -> None: ...
