import cuvslam as vslam
import os
import json
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from dataset_utils import read_stereo_edex
from PIL import Image

# generate pseudo-random colour from integer identifier for visualization
def color_from_id(identifier): return [(identifier * 17) % 256, (identifier * 31) % 256, (identifier * 47) % 256]

### setup rerun visualizer
rr.init('multicamera_hawk', strict=True, spawn=True)  # launch re-run instance
# setup rerun views
rr.send_blueprint(rrb.Blueprint(rrb.TimePanel(state="collapsed"),
                                rrb.Vertical(
                                    contents=[
                                        rrb.Horizontal(
                                            contents=[rrb.Spatial2DView(origin='car/cam0', name='front-stereo_left'),
                                                      rrb.Spatial2DView(origin='car/cam1', name='front-stereo_right'),
                                                      rrb.Spatial2DView(origin='car/cam2', name='back-stereo_left'),
                                                      rrb.Spatial2DView(origin='car/cam3',  name='back-stereo_right')]),
                                        rrb.Spatial3DView(name="3D", defaults=[rr.components.ImagePlaneDistance(0.5)]),
                                        rrb.Horizontal(
                                            contents=[rrb.Spatial2DView(origin='car/cam4', name='left-stereo_left'),
                                                      rrb.Spatial2DView(origin='car/cam5', name='left-stereo_right'),
                                                      rrb.Spatial2DView(origin='car/cam6', name='right-stereo_left'),
                                                      rrb.Spatial2DView(origin='car/cam7',  name='right-stereo_right')])
                                        ]
                                    ), 
                                ),
                            make_active=True)
# setup coordinate basis for root, cuvslam uses right-hand system with  X-right, Y-down, Z-forward
rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

# Load frame metadata
with open(os.path.join('datasets/r2b_galileo_edex/frame_metadata.jsonl'), 'r') as f:
    frames_metadata = [json.loads(i) for i in f.readlines()]

# Load camera configuration from EDEX file
cameras = read_stereo_edex('datasets/r2b_galileo_edex/stereo.edex')

# Set up VSLAM rig and tracker
rig = vslam.Rig()
rig.cameras = cameras

cfg = vslam.TrackerConfig(enable_final_landmarks_export = True,
                          odometry_mode = vslam.TrackerOdometryMode.Multicamera)

tracker = vslam.Tracker(rig, cfg)

trajectory = []

# Process each frame
for frame_id, frame in enumerate(frames_metadata):
    timestamp = max([i['timestamp'] for i in frame['cams']])
    images = [np.asarray(Image.open(os.path.join('datasets', 'r2b_galileo_edex', i['filename']))) for i in frame['cams']]
    # do multicamera visual tracking
    pose_estimate = tracker.track(timestamp, images)
    # get visualization data
    observations = [tracker.get_last_observations(i) for i in range(8)]
    landmarks = tracker.get_last_landmarks()
    final_landmarks = tracker.get_final_landmarks()
    # prepare visualization data
    observations_uv = [[[o.u, o.v] for o in obs_instance] for obs_instance in observations]
    observations_colors = [[color_from_id(o.id) for o in obs_instance] for obs_instance in observations]
    landmark_xyz = [l.coords for l in landmarks]
    landmarks_colors = [color_from_id(l.id) for l in landmarks]
    trajectory.append(pose_estimate.pose.translation)
    # send results to rerun for visualization
    rr.set_time_sequence('frame', frame_id)
    rr.log('trajectory', rr.LineStrips3D(trajectory))
    rr.log('final_landmarks', rr.Points3D(list(final_landmarks.values()), radii=0.01))
    rr.log('car', rr.Transform3D(translation=pose_estimate.pose.translation, quaternion=pose_estimate.pose.rotation))
    rr.log('car/body', rr.Boxes3D(centers=[0, 0.3 / 2, 0], sizes=[[0.35, 0.3, 0.66]]))
    rr.log('car/landmarks_center', rr.Points3D(landmark_xyz, radii=0.02, colors=landmarks_colors))

    for i in range(len(cameras)):
        rr.log('car/cam%s/image' % i, rr.Image(images[i]).compress(jpeg_quality=80))
        rr.log('car/cam%s/observations' % i, rr.Points2D(observations_uv[i], radii=5, colors=observations_colors[i]))

        # show only even cameras in 3D world
        if not i%2:
            rr.log('car/cam%s' % i, rr.Transform3D(translation=cameras[i].rig_from_camera.translation, 
                                                        rotation=rr.Quaternion(xyzw=cameras[i].rig_from_camera.rotation),
                                                        from_parent=False))
            rr.log('car/cam%s' % i, rr.Pinhole(image_plane_distance=1.,
                                            image_from_camera=np.array([[cameras[i].focal[0], 0, cameras[i].principal[0]],
                                                                        [0, cameras[i].focal[1], cameras[i].principal[1]],
                                                                        [0, 0, 1]]),
                                            width=cameras[i].size[0], height=cameras[i].size[1]))
