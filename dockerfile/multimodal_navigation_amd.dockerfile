FROM nvidia/cuda:11.7.1-devel-ubuntu22.04
ENV DEBIAN_FRONTEND noninteractive

# For xorg graphics
ENV QT_X11_NO_MITSHM 1

ARG USERNAME=nonroot
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Contact info
LABEL maintainer="Raul Castilla Arquillo <raulcastar@uma.es>"
LABEL authors="Raul Castilla Arquillo <raulcastar@uma.es>,\
Carlos Jesus Perez del Pulgar Mancebo <carlosperez@uma.es>"
LABEL organization="Space Robotics Laboratory (University of Malaga)"
LABEL url="https://www.uma.es/space-robotics"
LABEL version="1.0"
LABEL license="MIT License"
LABEL description=""
LABEL created=""

# Install necessary software for the installation of ROS2
RUN apt-get update && apt-get install -y \ 
                      locales \
                      curl \
                      gnupg2 \
                      lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

# Set the locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Add key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-humble-desktop python3-colcon-common-extensions

# Visual and dev packages
RUN apt-get update && \
    apt-get -y install python3.10  \
    libopencv-dev python3-opencv wget \
    libcanberra-gtk-module libcanberra-gtk3-module \
    tmux xorg nano vim curl python3-gi-cairo python3-pip \
    ffmpeg python3-tk git-all chafa \
    && rm -rf /var/lib/apt/lists/*

# Packages needed to run neural networks
RUN pip3 install torch torchvision einops timm matplotlib
RUN pip3 install argcomplete

# Update the package list, install sudo, create a non-root user, and grant password-less sudo permissions
RUN apt update && \
    apt install -y sudo && \
    addgroup --gid $USER_GID $USERNAME && \
    adduser --uid $USER_UID --gid $USER_GID --disabled-password --gecos "" $USERNAME && \
    echo "nonroot ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Install realsense camera drivers
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update && \
    apt-get -y install librealsense2-dkms librealsense2-utils  librealsense2-dev librealsense2-dbg \
    && rm -rf /var/lib/apt/lists/*

# Install packages for thermal camera
RUN apt-get update && \
    apt-get -y install freeglut3-dev libusb-1.0-0-dev guvcview \
    && rm -rf /var/lib/apt/lists/

# Install grid-map humble extension
# https://github.com/ethz-asl/grid_map
RUN apt-get update && \
    apt-get -y install ros-humble-grid-map \
    && rm -rf /var/lib/apt/lists/

# Install tkinter to avoid matplotlib graphs gui problems
RUN pip3 install tk

COPY ./dockerfile/88-irimager-usb.rules /etc/udev/

# Removes colcon build setup.py warning
RUN pip3 install setuptools==58.2.0 

# Set the non-root user as the default user
USER $USERNAME

# Set the working directory
WORKDIR /home/$USERNAME/

# Set ros environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
