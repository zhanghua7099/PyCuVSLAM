FROM nvcr.io/nvidia/cuda:12.6.1-runtime-ubuntu22.04

RUN apt-get update \
    && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends --allow-change-held-packages \
        libgtk-3-dev \
        libpython3.10 \
        libxkbcommon-x11-0 \
        python3-pip \
        unzip \
        vulkan-tools \
        wget \
        git \
        cmake \
        build-essential \
    && apt-get clean

RUN pip3 install --upgrade pip

ARG LIBREALSENSE_SOURCE_VERSION=v2.55.1

COPY scripts/build-librealsense.sh /opt/realsense/build-librealsense.sh
COPY scripts/install-realsense-dependencies.sh /opt/realsense/install-realsense-dependencies.sh

RUN chmod +x /opt/realsense/install-realsense-dependencies.sh && \
    /opt/realsense/install-realsense-dependencies.sh; \
    chmod +x /opt/realsense/build-librealsense.sh && /opt/realsense/build-librealsense.sh -v ${LIBREALSENSE_SOURCE_VERSION};

# Copy hotplug script for udev rules/hotplug for RealSense
RUN mkdir -p /opt/realsense/
COPY scripts/hotplug-realsense.sh /opt/realsense/hotplug-realsense.sh
COPY udev_rules/99-realsense-libusb-custom.rules /etc/udev/rules.d/99-realsense-libusb-custom.rules

WORKDIR /pycuvslam