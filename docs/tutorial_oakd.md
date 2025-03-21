# Tutorial: Running PyCuVSlam Stereo Visual Odometry on OAK-D Stereo Camera

This tutorial shows how to run live pyCUVSLAM tracking on an OAK-D stereo camera that provides unrectified images

## Setting up the cuvslam environment
Refer to the [Installation Guide](../README.md#Installation-Guide) for instructions on installing and configuring all required dependencies

## Setting up DepthAI
1. Install the [DepthAI Python library](https://github.com/luxonis/depthai-python) following the official documentation
2. Test your setup by running a basic [camera example](https://docs.luxonis.com/software/depthai/examples/rgb_preview/). Ensure it works correctly before proceeding

## Running Stereo Visual Odometry

```bash
python3 examples/oak-d/run_stereo.py
```

You should see the following interactive visualization in rerun: 
![Visualization Example](tutorial_oakd_stereo.gif)

Note: The pyCUVSLAM stereo tracker expects reliably synchronized stereo pairs with a stable FPS. If your camera pipeline is doing extensive on-device processing or AI inference, frame rates may drop, and image pairs may become unsynchronized. Watch for warnings about low FPS or mismatched stereo frames

If you experience low FPS even in the basic setup, you can investigate potential bottlenecks using the supplied [measurement tools](https://docs.luxonis.com/software/depthai/optimizing/) for OAK devices