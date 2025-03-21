import cuvslam as vslam
import numpy as np
import pyrealsense2 as rs
from camera_utils import get_rs_stereo_rig, to_odometry_mode
from visualizer import RerunVisualizer

# Initialize RealSense configuration
config = rs.config()
pipeline = rs.pipeline()

# Configure streams
config.enable_stream(rs.stream.infrared, 1, 640, 360, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 360, rs.format.y8, 30)

# Start pipeline to get intrinsics and extrinsics
profile = pipeline.start(config)
frames = pipeline.wait_for_frames()
pipeline.stop()

# Prepare camera parameters
camera_params = {'left': {}, 'right': {}}

# Get extrinsics and intrinsics
camera_params['left']['intrinsics'] = frames[0].profile.as_video_stream_profile().intrinsics
camera_params['right']['intrinsics'] = frames[1].profile.as_video_stream_profile().intrinsics
camera_params['right']['extrinsics'] = frames[1].profile.get_extrinsics_to(frames[0].profile)

# Configure tracker
cfg = vslam.TrackerConfig(async_sba=False, enable_final_landmarks_export=True, 
                          odometry_mode = to_odometry_mode('stereo'), horizontal_stereo_camera=True)

# Create rig using utility function
rig = get_rs_stereo_rig(camera_params)

# Initialize tracker and visualizer
tracker = vslam.Tracker(rig, cfg)

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

# Disable IR emitter if supported
depth_sensor = device.query_sensors()[0]
if depth_sensor.supports(rs.option.emitter_enabled):
    depth_sensor.set_option(rs.option.emitter_enabled, 0)

visualizer = RerunVisualizer()

# Start pipeline for tracking
profile = pipeline.start(config)

frame_id = 0

prev_timestamp = None
threshold_ns = 35 * 1e6  # 35ms in nanoseconds

trajectory = []

try:

    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
    
        lframe = frames.get_infrared_frame(1)
        rframe = frames.get_infrared_frame(2)
        if not lframe or not rframe:
            continue
            
        timestamp = int(lframe.timestamp * 1e6)  # Convert to nanoseconds
        
        # Check timestamp difference with previous frame
        if prev_timestamp is not None:
            timestamp_diff = timestamp - prev_timestamp
            if timestamp_diff > threshold_ns:
                print(f"Warning: Camera stream message drop: timestamp gap ({timestamp_diff/1e6:.2f} ms) exceeds threshold {threshold_ns/1e6:.2f} ms")
        
        # Store current timestamp for next iteration
        prev_timestamp = timestamp
        
        images = (np.asanyarray(lframe.get_data()), np.asanyarray(rframe.get_data()))

        # Track frame
        pose_estimate = tracker.track(timestamp, images)

        frame_id += 1
        trajectory.append(pose_estimate.pose.translation)

        # Visualize results
        visualizer.visualize_frame(
            frame_id=frame_id,
            images=images,
            pose=pose_estimate.pose,
            observations_main_cam=tracker.get_last_observations(0),
            trajectory=trajectory,
            timestamp=timestamp
        )

finally:
    pipeline.stop()
