# Tutorial: Running PyCuVSLAM Stereo Visual Odometry on the KITTI dataset

The KITTI Vision Benchmark Suite is a high-quality dataset benchmarking and comparing various computer vision algorithms.
Among other options, the KITTI dataset has sequences for evaluating stereo-visual odometry.

## System Requirements
Before going to the next step, make sure you've setup your system to support PyCuVSLAM: [System Requirements](https://gitlab-master.nvidia.com/elbrus/pycuvslam/-/tree/main#system-requirements)


## Dataset Setup

1. To get the KITTI test sequences, download 
[the odometry data set](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) (grayscale, 22 GB).
You will have to register for an account. Once you do, you will receive a download link to the `data_odometry_gray.zip` file.

2. Unpack it to your dataset folder.

    ```bash
    unzip data_odometry_gray.zip -d examples/kitti
    ```

    When you unpack it, you get the
    following structure:
    ```
    dataset_odometry_gray
       dataset  -> should be mapped or placed in examples/kitti/dataset
          sequences
              00
                 image_0
                    000000.png
                      ...
                    004550.png  
                 image_1
                    000000.png
                      ...
                    004550.png  
                 calib.txt
                 times.txt
              01  
                 ...
    ```
    Each of the 20 folders has a stereo sequence and calibration of cameras.

## Running PyCuVSLAM

Execute the following command to start tracking on KITTI:

```bash
python3 examples/kitti/track_kitti.py
```

## Visualization

The visualization will look something like this:

![Visualization Example](tutorial_kitti.jpg)
