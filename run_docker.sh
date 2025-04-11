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
  --network host \
  --device=/dev/bus/usb \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=$XAUTH \
  -v $XSOCK:$XSOCK \
  -v $XAUTH:$XAUTH \
  -v .:/pycuvslam \
  -v "$DATASETS":"$DATASETS" \
  pycuvslam