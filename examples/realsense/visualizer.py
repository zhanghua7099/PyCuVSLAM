from typing import List
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import cuvslam as vslam


class RerunVisualizer:
    def __init__(self):
        """Initialize rerun visualizer."""
        rr.init("cuVSLAM Visualizer", spawn=True)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
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
            ), 
            make_active=True
        )
        self.track_colors = {}

    def _log_rig_pose(self, rotation_quat: np.ndarray,
                      translation: np.ndarray) -> None:
        """Log rig pose to Rerun."""
        scale = 0.1
        rr.log(
            "world/camera_0",
            rr.Transform3D(translation=translation, quaternion=rotation_quat),
            rr.Arrows3D(
                vectors=np.eye(3) * scale,
                colors=[[255,0,0], [0,255,0], [0,0,255]]  # RGB for XYZ axes
            )
        )

    def _log_observations(self, observations_main_cam: List[vslam.Observation],
                         image: np.ndarray) -> None:
        """Log 2D observations for a specific camera with consistent colors per track."""
        if not observations_main_cam:
            return

        # Assign random color to new tracks
        for obs in observations_main_cam:
            if obs.id not in self.track_colors:
                self.track_colors[obs.id] = np.random.randint(0, 256, size=3)

        points = np.array([[obs.u, obs.v] for obs in observations_main_cam])
        colors = np.array([self.track_colors[obs.id] for obs in observations_main_cam])
            
        rr.log(
            "world/camera_0/observations",
            rr.Points2D(positions=points, colors=colors, radii=5.0),
            rr.Image(image).compress()
        )

    def _log_gravity(self, gravity: np.ndarray) -> None:
        """Log gravity vector to Rerun."""
        scale = 0.02
        rr.log(
            "world/camera_0/gravity",
            rr.Arrows3D(vectors=gravity, colors=[[255, 0, 0]], radii=scale)
        )

    def visualize_frame(self, frame_id: int, images: List[np.ndarray], 
                       pose: vslam.Pose, observations_main_cam: List[vslam.Observation],
                       trajectory: List[np.ndarray], timestamp: int, gravity: np.ndarray = None) -> None:
        """Visualize current frame state using Rerun."""
        rr.set_time_sequence("frame", frame_id)
        rr.log("world/trajectory", rr.LineStrips3D(trajectory), static=True)
        for camera_idx, image in enumerate(images):
            # Visualize main camera
            if camera_idx == 0:
                self._log_rig_pose(pose.rotation, pose.translation)
                self._log_observations(observations_main_cam, image)
                if gravity is not None:
                    self._log_gravity(gravity)
        rr.log("world/timestamp", rr.TextLog(str(timestamp)))
