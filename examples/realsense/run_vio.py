import pyrealsense2 as rs
import numpy as np
import cuvslam as vslam
import queue
import threading
from camera_utils import get_rs_vio_rig, to_odometry_mode
from copy import deepcopy
from visualizer import RerunVisualizer

class ThreadWithTimestamp:
    def __init__(self, low_rate_threshold_ns, high_rate_threshold_ns):
        self.prev_low_rate_timestamp = None
        self.prev_high_rate_timestamp = None
        self.low_rate_threshold_ns = low_rate_threshold_ns
        self.high_rate_threshold_ns = high_rate_threshold_ns
        self.last_low_rate_timestamp = None

def imu_thread(tracker, q, thread_with_timestamp, motion_pipe):
    while True:
        imu_measurement = vslam.ImuMeasurement()
        imu_frames = motion_pipe.wait_for_frames()        
        current_timestamp = int(imu_frames[0].timestamp*1e6)
        timestamp_diff = 0

        # Receive last available timestamp from low_rate_thread and print warning if necessary
        if thread_with_timestamp.last_low_rate_timestamp is not None and current_timestamp < thread_with_timestamp.last_low_rate_timestamp:
            print(f"Warning: The IMU stream timestamp is earlier than the previous timestamp from the Camera stream ({current_timestamp} < {thread_with_timestamp.last_low_rate_timestamp})")
            continue

        # Store own previous timestamp and compare with threshold
        if thread_with_timestamp.prev_high_rate_timestamp is not None:
            timestamp_diff = current_timestamp - thread_with_timestamp.prev_high_rate_timestamp
            if timestamp_diff > thread_with_timestamp.high_rate_threshold_ns:
                print(f"Warning: IMU stream message drop: timestamp gap ({timestamp_diff/1e6:.2f} ms) exceeds threshold {thread_with_timestamp.high_rate_threshold_ns/1e6:.2f} ms")
            elif timestamp_diff < 0:
                print(f"Warning: IMU messages are not sequential")

        if timestamp_diff < 0:
            continue
        
        thread_with_timestamp.prev_high_rate_timestamp = deepcopy(current_timestamp)
        imu_measurement.timestamp_ns = current_timestamp
        imu_measurement.linear_accelerations = np.frombuffer(imu_frames[0].get_data(), dtype=np.float32)[:3]
        imu_measurement.angular_velocities = np.frombuffer(imu_frames[1].get_data(), dtype=np.float32)[:3]

        if timestamp_diff > 0:
            tracker.register_imu_measurement(0, imu_measurement)

def camera_thread(tracker, q, thread_with_timestamp, ir_pipe):
    while True:
        ir_frames = ir_pipe.wait_for_frames()
        ir_lframe = ir_frames.get_infrared_frame(1)
        ir_rframe = ir_frames.get_infrared_frame(2)
        current_timestamp = int(ir_lframe.timestamp*1e6)
        
        # Store own previous timestamp and compare with threshold
        if thread_with_timestamp.prev_low_rate_timestamp is not None:
            timestamp_diff = current_timestamp - thread_with_timestamp.prev_low_rate_timestamp
            if timestamp_diff > thread_with_timestamp.low_rate_threshold_ns:
                print(f"Warning: Camera stream message drop: timestamp gap ({timestamp_diff/1e6:.2f} ms) exceeds threshold {thread_with_timestamp.low_rate_threshold_ns/1e6:.2f} ms")
        
        thread_with_timestamp.prev_low_rate_timestamp = deepcopy(current_timestamp)
        images = (np.asanyarray(ir_lframe.get_data()), np.asanyarray(ir_rframe.get_data()))
        
        pose_estimate = tracker.track(current_timestamp, images)
        q.put([current_timestamp, pose_estimate, images])  # Put the output in the queue for other processes
        thread_with_timestamp.last_low_rate_timestamp = current_timestamp

# Initialize RealSense configuration
config = rs.config()
pipeline = rs.pipeline()

# Configure streams for initial setup
config.enable_stream(rs.stream.infrared, 1, 640, 360, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 360, rs.format.y8, 30)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

# Start pipeline to get intrinsics and extrinsics
profile = pipeline.start(config)
frames = pipeline.wait_for_frames()
pipeline.stop()

# Prepare camera parameters
camera_params = {'left': {}, 'right': {}, 'imu': {}}

# Get extrinsics and intrinsics
camera_params['right']['extrinsics'] = frames[1].profile.get_extrinsics_to(frames[0].profile)
camera_params['imu']['cam_from_imu'] = frames[2].profile.get_extrinsics_to(frames[0].profile)
camera_params['left']['intrinsics'] = frames[0].profile.as_video_stream_profile().intrinsics
camera_params['right']['intrinsics'] = frames[1].profile.as_video_stream_profile().intrinsics

# Configure tracker
cfg = vslam.TrackerConfig(async_sba=False, enable_final_landmarks_export=True, debug_imu_mode = False, 
                          odometry_mode = to_odometry_mode('inertial'), horizontal_stereo_camera=True)

# Create rig using utility function
rig = get_rs_vio_rig(camera_params)

# Initialize tracker
tracker = vslam.Tracker(rig, cfg)

# Set up IR pipeline
ir_pipe = rs.pipeline()
ir_config = rs.config()
ir_config.enable_stream(rs.stream.infrared, 1, 640, 360, rs.format.y8, 30)
ir_config.enable_stream(rs.stream.infrared, 2, 640, 360, rs.format.y8, 30)

# Get device info
ir_wrapper = rs.pipeline_wrapper(ir_pipe)
ir_profile = config.resolve(ir_wrapper)
device = ir_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Disable IR emitter if supported
depth_sensor = device.query_sensors()[0]
if depth_sensor.supports(rs.option.emitter_enabled):
    depth_sensor.set_option(rs.option.emitter_enabled, 0)

# Set up motion pipeline
motion_pipe = rs.pipeline()
motion_config = rs.config()
motion_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
motion_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

# Start pipelines
motion_profile = motion_pipe.start(motion_config)
ir_profile = ir_pipe.start(ir_config)

# Set up threading and visualization
q = queue.Queue()
camera_threshold_ns = 35*1e6  # Define threshold in nanoseconds for low rate thread
imu_threshold_ns = 5.5*1e6

visualizer = RerunVisualizer()
thread_with_timestamp = ThreadWithTimestamp(camera_threshold_ns, imu_threshold_ns)

# Start threads
t1 = threading.Thread(target=imu_thread, args=(tracker, q, thread_with_timestamp, motion_pipe))
t2 = threading.Thread(target=camera_thread, args=(tracker, q, thread_with_timestamp, ir_pipe))

t1.start()
t2.start()

frame_id = 0
prev_timestamp = None

trajectory = []
try:
    while True:
        
        # Get the output from the queue
        timestamp, pose_estimate, images = q.get()  

        frame_id += 1
        trajectory.append(pose_estimate.pose.translation)

        gravity = None
        if cfg.odometry_mode == vslam.TrackerOdometryMode.Inertial:
            # Gravity estimation requires collecting sufficient number of keyframes with motion diversity
            # Get gravity in rig frame
            gravity = tracker.get_last_gravity()

        # Visualize results
        visualizer.visualize_frame(
            frame_id=frame_id,
            images=images,
            pose=pose_estimate.pose,
            observations_main_cam=tracker.get_last_observations(0),
            trajectory=trajectory,
            timestamp=timestamp,
            gravity=gravity
        )
        
finally:
    motion_pipe.stop()
    ir_pipe.stop()