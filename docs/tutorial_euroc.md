# Tutorial: Running PyCuVSLAM on the EuRoC MAV dataset

This tutorial demonstrates how to use PyCuVSLAM with the EuRoC MAV dataset.

## System Requirements
Before going to the next step, make sure you've setup your system to support PyCuVSLAM: [System Requirements](https://gitlab-master.nvidia.com/elbrus/pycuvslam/-/tree/main#system-requirements)

## Dataset Setup

1. Download the EuRoC MH_01_easy dataset:

    ```bash
    wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip -O examples/euroc/dataset/MH_01_easy.zip
    unzip examples/euroc/dataset/MH_01_easy.zip -d examples/euroc/dataset
    rm examples/euroc/dataset/MH_01_easy.zip
    ```

2. Copy the calibration files:

    ```bash
    cp examples/euroc/dataset/sensor_cam0.yaml examples/euroc/dataset/mav0/cam0/sensor_recalibrated.yaml
    cp examples/euroc/dataset/sensor_cam1.yaml examples/euroc/dataset/mav0/cam1/sensor_recalibrated.yaml
    cp examples/euroc/dataset/sensor_imu0.yaml examples/euroc/dataset/mav0/imu0/sensor_recalibrated.yaml
    ```

> **Note**: We provide recalibrated camera and IMU parameters to improve tracking accuracy. We strongly recommend using these calibration files instead of the original ones provided with the dataset.

## Running PyCuVSLAM

Execute the following command to start tracking on EuRoC:

```bash
python3 examples/euroc/track_euroc.py
```

### Available Modes

To change tracking mode, modify this line in `track_euroc.py`:
```python
# Available tracking modes:
#   vslam.TrackerOdometryMode.Mono        - Monocular visual odometry
#   vslam.TrackerOdometryMode.Multicamera  - Stereo visual odometry
#   vslam.TrackerOdometryMode.Inertial    - Visual-inertial odometry
euroc_tracking_mode = vslam.TrackerOdometryMode.Mono  # Change this line for desired mode
```

## Visualization

The real-time visualization shows:
- Camera trajectory
- Feature tracking
- Gravity vector (in inertial mode)

![Visualization Example](tutorial_euroc.jpg)








