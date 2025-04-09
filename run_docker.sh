#!/bin/bash
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
DATASETS=$(realpath -s ~/datasets)
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH
docker run -it \
  --runtime=nvidia \
  --rm \
  --gpus all \
  --privileged \
  --device=/dev/bus/usb \
  --network host \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=$XAUTH \
  -v $XSOCK:$XSOCK \
  -v $XAUTH:$XAUTH \
  -v .:/pycuvslam \
  -v "$DATASETS":"$DATASETS" \
  pycuvslam
