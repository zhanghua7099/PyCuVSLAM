# Tutorial: Running PycuVSLAM Stereo Visual Odometry on a Realsense Stereo Camera

This tutorial walks you through how to operate live PycuVSLAM tracking on a Realsense stereo camera

## Set Up the pyCUVSLAM Environment

Refer to the [Installation Guide](../README.md#Installation-Guide) for detailed environment setup instructions

## Setting Up librealsense

The official [librealsense Python installation guide](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/readme.md) is recommended. We highly suggest building both librealsense and its Python wrapper from source:

1. Follow the build instructions for the [librealsense library](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

2. Compile librealsense2 with Python bindings:
   ```bash
   cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true -DPYTHON_EXECUTABLE=$(which python3.10)

   make -j
   ```

3. Once the build and installation are finished, add the build directory to your path:
   ```bash
   export PYTHONPATH=$PYTHONPATH:~/librealsense/build/Release
   ```

4. Test your setup by running a [simple camera example](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py) to verify that everything was installed correctly

## Running Stereo Visual Odometry

Go to the `examples/realsense` folder and run stereo visual odometry:
```bash
python3 run_stereo.py
```
You should see the following interactive visualization in rerun:
![Visualization Example](tutorial_realsense_stereo.gif)

## Running Stereo Inertial Odometry

Stereo inertial odometry is only supported by Realsense stereo cameras equipped with an onboard IMU


Note: Stereo Inertial Odometry relies heavily on accurate [IMU Noise Parameters](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model). You should [calculate IMU noise parameters](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) for your specific camera and update the settings in [camera_utils.py](../examples/realsense/camera_utils.py) accordingly

Run stereo visual inertial odometry:
```bash
python3 run_vio.py
```
After a brief moment, a gravity vector (indicated by the bold red line pointing downward) should appear in the 3D visualization window:
![Visualization Example](tutorial_realsense_vio.gif)