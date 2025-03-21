import os

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

import cuvslam as vslam
from dataset_utils import prepare_frame_metadata_euroc, get_rig, load_frame

# Available tracking modes:
#   vslam.TrackerOdometryMode.Mono        - Monocular visual odometry
#   vslam.TrackerOdometryMode.Multicamera - Stereo visual odometry
#   vslam.TrackerOdometryMode.Inertial    - Visual-inertial odometry

euroc_tracking_mode = vslam.TrackerOdometryMode.Inertial

# Setup rerun visualizer
rr.init("cuVSLAM Visualizer", spawn=True)

# Setup coordinate basis for root, cuvslam uses right-hand system with  X-right, Y-down, Z-forward
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

# Setup rerun views
rr.send_blueprint(
    rrb.Blueprint(
        rrb.TimePanel(state="collapsed"),
        rrb.Horizontal(
            column_shares=[0.5, 0.5],
            contents=[
                rrb.Spatial2DView(origin='world/camera_0'),
                rrb.Spatial3DView(origin='world'),

            ]
        )
    )
)


# Generate pseudo-random color from integer identifier for visualization
def color_from_id(identifier): return [(identifier * 17) % 256, (identifier * 31) % 256, (identifier * 47) % 256]


# Prepare frames metadata
euroc_dataset_path = os.path.join(os.path.dirname(__file__), "dataset/mav0")
frames_metadata = prepare_frame_metadata_euroc(euroc_dataset_path, euroc_tracking_mode)

# Get camera rig
rig = get_rig(euroc_dataset_path)

# Configure tracker
cfg = vslam.TrackerConfig()
cfg.odometry_mode = euroc_tracking_mode
cfg.async_sba = False
cfg.enable_observations_export = True

# Initialize tracker
tracker = vslam.Tracker(rig, cfg)

# Track frames
last_camera_timestamp = None
imu_count_since_last_camera = 0
frame_id = 0
trajectory = []
for frame_metadata in frames_metadata:
    timestamp = frame_metadata['timestamp']
    if frame_metadata['type'] == 'imu':
        imu_measurement = vslam.ImuMeasurement()
        imu_measurement.timestamp_ns = int(timestamp)
        imu_measurement.linear_accelerations = np.array(frame_metadata['accel'])
        imu_measurement.angular_velocities = np.array(frame_metadata['gyro'])
        tracker.register_imu_measurement(0, imu_measurement)
        imu_count_since_last_camera += 1
        continue

    images = [load_frame(image_path) for image_path in frame_metadata['images_paths']]

    # Check IMU measurements before tracking
    if cfg.odometry_mode == vslam.TrackerOdometryMode.Inertial and last_camera_timestamp is not None:
        if imu_count_since_last_camera == 0:
            print(f"Warning: No IMU measurements between timestamps {last_camera_timestamp} and {timestamp}")

    # Reset counters
    last_camera_timestamp = timestamp
    imu_count_since_last_camera = 0

    # Track frame
    pose_estimate = tracker.track(timestamp, images)
    trajectory.append(pose_estimate.pose.translation)

    # Visualize

    # Get current pose and observations for the main camera and gravity in rig frame
    current_pose = pose_estimate.pose
    current_observations_main_cam = tracker.get_last_observations(0)
    gravity = None
    if cfg.odometry_mode == vslam.TrackerOdometryMode.Inertial:
        # Gravity estimation requires collecting sufficient number of keyframes with motion diversity
        gravity = tracker.get_last_gravity()

    rr.set_time_sequence("frame", frame_id)
    rr.log("world/trajectory", rr.LineStrips3D(trajectory), static=True)
    rr.log(
        "world/camera_0",
        rr.Transform3D(translation=current_pose.translation, quaternion=current_pose.rotation),
        rr.Arrows3D(
            vectors=np.eye(3) * 0.2,
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]  # RGB for XYZ axes
        )
    )

    points = np.array([[obs.u, obs.v] for obs in current_observations_main_cam])
    colors = np.array([color_from_id(obs.id) for obs in current_observations_main_cam])
    rr.log(
        "world/camera_0/observations",
        rr.Points2D(positions=points, colors=colors, radii=5.0),
        rr.Image(images[0]).compress(jpeg_quality=80)
    )
    if gravity is not None:
        rr.log(
            "world/camera_0/gravity",
            rr.Arrows3D(vectors=gravity, colors=[[255, 0, 0]], radii=0.015)
        )

    frame_id += 1
