from os.path import join

import rerun as rr
import rerun.blueprint as rrb
from PIL import Image
from numpy import loadtxt, asarray

import cuvslam

# Dataset sequence to track and visualize
sequence_path = 'examples/kitti/dataset/sequences/06'


# Generate pseudo-random colour from integer identifier for visualization
def color_from_id(identifier): return [(identifier * 17) % 256, (identifier * 31) % 256, (identifier * 47) % 256]


# Setup rerun visualizer
rr.init('kitti', strict=True, spawn=True)  # launch re-run instance

# Setup rerun views
rr.send_blueprint(rrb.Blueprint(rrb.TimePanel(state="collapsed"),
                                rrb.Vertical(row_shares=[0.6, 0.4],
                                             contents=[rrb.Spatial3DView(), rrb.Spatial2DView(origin='car/cam0')])))

# Setup coordinate basis for root, cuvslam uses right-hand system with  X-right, Y-down, Z-forward
rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

# Draw arrays in origin X-red, Y-green, Z-blue
rr.log("xyz", rr.Arrows3D(vectors=[[50, 0, 0], [0, 50, 0], [0, 0, 50]], colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                          labels=['[x]', '[y]', '[z]']), static=True)

# Load KITTI dataset (calibration, timestamps and image sizes)
# Load projection matrices per camera [P0, P1, P2, P3]
intrinsics = loadtxt(join(sequence_path, 'calib.txt'), usecols=range(1, 13)).reshape(4, 3, 4)
size = Image.open(join(sequence_path, 'image_0', '000000.png')).size  # assume all images have the same size
timestamps = [int(10 ** 9 * float(sec_str)) for sec_str in open(join(sequence_path, 'times.txt')).readlines()]

# Initialize the cuvslam tracker
cameras = [cuvslam.Camera(), cuvslam.Camera()]
for i in [0, 1]:
    cameras[i].size = size
    cameras[i].principal = [intrinsics[i][0][2], intrinsics[i][1][2]]
    cameras[i].focal = [intrinsics[i].diagonal()[0], intrinsics[i].diagonal()[1]]
cameras[1].rig_from_camera.translation[0] = - intrinsics[1][0][3] / intrinsics[1][0][0]  # stereo-camera baseline
cfg = cuvslam.TrackerConfig(async_sba=False, enable_final_landmarks_export=True, horizontal_stereo_camera=True)
tracker = cuvslam.Tracker(cuvslam.Rig(cameras), cfg)

# Track each frames in the dataset sequence
trajectory = []
for frame in range(len(timestamps)):
    # Load grayscale pixels as array for left and right absolute image paths
    images = [asarray(Image.open(join(sequence_path, f'image_{cam}', f'{frame:0>6}.png'))) for cam in [0, 1]]

    # Do tracking
    result = tracker.track(timestamps[frame], images)

    # Get visualization data
    observations = tracker.get_last_observations(0)  # get observation from left camera
    landmarks = tracker.get_last_landmarks()
    final_landmarks = tracker.get_final_landmarks()

    # Prepare visualization data
    observations_uv = [[o.u, o.v] for o in observations]
    observations_colors = [color_from_id(o.id) for o in observations]
    landmark_xyz = [l.coords for l in landmarks]
    landmarks_colors = [color_from_id(l.id) for l in landmarks]
    trajectory.append(result.pose.translation)

    # Send results to rerun for visualization
    rr.set_time_sequence('frame', frame)
    rr.log('trajectory', rr.LineStrips3D(trajectory))
    rr.log('final_landmarks', rr.Points3D(list(final_landmarks.values()), radii=0.1))
    rr.log('car', rr.Transform3D(translation=result.pose.translation, quaternion=result.pose.rotation))
    rr.log('car/body', rr.Boxes3D(centers=[0, 1.65 / 2, 0], sizes=[[1.6, 1.65, 2.71]]))
    rr.log('car/landmarks_center', rr.Points3D(landmark_xyz, radii=0.25, colors=landmarks_colors))
    rr.log('car/landmarks_lines', rr.Arrows3D(vectors=landmark_xyz, radii=0.05, colors=landmarks_colors))
    rr.log('car/cam0', rr.Pinhole(image_plane_distance=1.68, image_from_camera=intrinsics[0][:3, :3],
                                  width=size[0], height=size[1]))
    rr.log('car/cam0/image', rr.Image(images[0]).compress(jpeg_quality=80))
    rr.log('car/cam0/observations', rr.Points2D(observations_uv, radii=5, colors=observations_colors))
