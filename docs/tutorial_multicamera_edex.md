# Tutorial: Running PyCuVSlam Multicamera Visual Odometry on the R2B Galileo dataset

cuVSLAM could be run in multicamera mode on rig with HW-syncronized stereo cameras. This example is based on original tutorial provided for ROSbag in [ISAAC ROS dicumentation](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/tutorial_multi_hawk.html)

## Get dataset
To get the multicamera datasets you need to download [r2b datasets in ROSbag format](https://registry.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/r2bdataset2024/files) and convert it using [edex extractor](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/isaac_ros_rosbag_utils/index.html#edex-extraction) 

1. Follow the original [ISAAC ROS Multi-camera Visual SLAM tutorial](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/tutorial_multi_hawk.html) of execution on a rosbag. Make sure, that rosbag file has been downloaded and you are able to play it within docker container:

```bash
ros2 bag play ${ISAAC_ROS_WS}/isaac_ros_assets/rosbags/r2b_galileo
```

2. Set up [Isaac ROS rosbag utils](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/isaac_ros_rosbag_utils/index.html) package and run within docker container:

```bash
ros2 run isaac_ros_rosbag_utils extract_edex --config_path src/isaac_ros_common/isaac_ros_rosbag_utils/config/edex_extraction_nova.yaml --rosbag_path ${ISAAC_ROS_WS}/isaac_ros_assets/rosbags/r2b_galileo --edex_path ${ISAAC_ROS_WS}/isaac_ros_assets/r2b_galileo_edex
```

The expected stucture of output folder:
```bash
r2b_galileo_edex
├── frame_metadata.jsonl
├── images
│   ├── back_stereo_camera
│   │   ├── left
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ...
│   │   └── right
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ...
│   ├── front_stereo_camera
│   │   ├── left
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ...
│   │   └── right
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ...
│   ├── left_stereo_camera
│   │   ├── left
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ...
│   │   └── right
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ...
│   ├── raw_timestamps.csv
│   ├── right_stereo_camera
│   │   ├── left
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ...
│   │   └── right
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ...
│   └── synced_timestamps.csv
├── robot.urdf
└── stereo.edex
```

3. Exit from docker container and move `r2b_galileo_edex` folder to multicamera example folder:
```bash
mkdir -p ~/pycuvslam/examples/multicamera_edex/datasets
mv ~/workspaces/isaac/isaac_ros_assets/r2b_galileo_edex ~/pycuvslam/examples/multicamera_edex/datasets/r2b_galileo_edex
```

## Setup test environment

Setup cuVSLAM virtual environment based on [Installation Guide](../README.md#Installation-Guide)

## Run tracking and visualization

Go to the `examples/multicamera_edex` folder and run

```bash
python3 track_multicamera.py
```

As a result, you will get rerun with fancy visualization:

![Visualization Example](tutorial_multicamera_edex.gif)
