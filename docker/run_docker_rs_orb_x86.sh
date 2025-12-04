#!/bin/bash

CONTAINER_NAME=pycuvslam_rs_orb

# Ensure X11 forwarding is set up properly
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

touch $XAUTH
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH

# Set up datasets path (create if doesn't exist)
DATASETS=$(realpath -s ~/datasets 2>/dev/null || echo ~/datasets)
mkdir -p "$DATASETS"

# Allow X11 connections from localhost
xhost +local:docker

echo "Checking if container '$CONTAINER_NAME' exists..."

# ======= CHECK IF CONTAINER EXISTS =======
EXISTS=$(docker ps -a --filter "name=^${CONTAINER_NAME}$" --format "{{.Names}}")

if [ "$EXISTS" == "$CONTAINER_NAME" ]; then
    echo "Container exists."

    # Check if running
    RUNNING=$(docker ps --filter "name=^${CONTAINER_NAME}$" --format "{{.Names}}")

    if [ "$RUNNING" == "$CONTAINER_NAME" ]; then
        echo "Container is already running. Attaching..."
        docker exec -it "$CONTAINER_NAME" bash
    else
        echo "Container exists but is stopped. Starting..."
        docker start "$CONTAINER_NAME"
        docker exec -it "$CONTAINER_NAME" bash
    fi

    exit 0
fi

echo "Container does not exist. Creating new one..."
echo "Starting PycuVSLAM Docker container with RealSense support..."
echo "Datasets directory: $DATASETS"

# ======== CREATE A NEW CONTAINER ========
docker run -it \
  --runtime=nvidia \
  --gpus all \
  --privileged \
  --network host \
  --name $CONTAINER_NAME \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=$XAUTH \
  -e QT_X11_NO_MITSHM=1 \
  -e _X11_NO_MITSHM=1 \
  -e _MITSHM=0 \
  -v $XSOCK:$XSOCK \
  -v $XAUTH:$XAUTH \
  -v .:/pycuvslam \
  -v "$DATASETS":"$DATASETS" \
  --device=/dev/video0:/dev/video0 \
  --device=/dev/video1:/dev/video1 \
  --device=/dev/video2:/dev/video2 \
  -v /dev/bus/usb:/dev/bus/usb \
  ghcr.io/zhanghua7099/pycuvslam:main \
  bash -c "
    # Install pycuvslam package in editable mode if it exists
    if [ -d '/pycuvslam/bin/x86_64' ]; then
        echo 'Installing pycuvslam package in editable mode for x86...'
        pip3 install -e /pycuvslam/bin/x86_64
        echo 'pycuvslam package installed successfully'
    else
        echo 'Warning: /pycuvslam/bin/x86_64 not found. Skipping pycuvslam installation.'
    fi

    exec bash
  "

# Clean up X11 permissions
xhost -local:docker
