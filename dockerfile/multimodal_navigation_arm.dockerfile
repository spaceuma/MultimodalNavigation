FROM nvcr.io/nvidia/l4t-jetpack:r36.2.0
ENV DEBIAN_FRONTEND noninteractive

# Jetson docker containers at https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags

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
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
    && rm -rf /var/lib/apt/lists/*

# Packages needed to run neural networks
RUN pip3 install einops timm matplotlib
RUN pip3 install argcomplete

# Update the package list, install sudo, create a non-root user, and grant password-less sudo permissions
RUN apt update && \
    apt install -y sudo && \
    addgroup --gid $USER_GID $USERNAME && \
    adduser --uid $USER_UID --gid $USER_GID --disabled-password --gecos "" $USERNAME && \
    echo "nonroot ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Set the working directory
WORKDIR /home/$USERNAME/

# Install realsense camera drivers
RUN git clone https://github.com/IntelRealSense/librealsense.git
RUN cd librealsense && mkdir build && cd build  && \
    cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=true -DBUILD_WITH_CUDA=false && \ 
    make -j5 && make install


# Install packages for thermal camera
RUN apt-get update && \
    apt-get -y install freeglut3-dev libusb-1.0-0-dev guvcview \
    && rm -rf /var/lib/apt/lists/

COPY ./dockerfile/88-irimager-usb.rules /etc/udev/rules.d/
COPY ./dockerfile/99-realsense-libusb.rules /etc/udev/rules.d/

# Cleaning older installs
RUN pip3 uninstall -y torch
RUN pip3 uninstall -y torchvision

# Installing jetson pytorch version
# https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
ENV FORCE_CUDA = 1 
ENV TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v60dp/pytorch/torch-2.2.0a0+6a974be.nv23.11-cp310-cp310-linux_aarch64.whl
RUN python3 -m pip install --no-cache $TORCH_INSTALL
RUN apt-get update && \
    apt-get -y install libopenblas-base libopenmpi-dev libomp-dev \
    && rm -rf /var/lib/apt/lists/

# Fix numpy error
RUN pip install numpy --upgrade

# Install torchvision
RUN apt-get update && \
    apt-get -y install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev \
    && rm -rf /var/lib/apt/lists/
RUN git clone --branch v0.17.0 https://github.com/pytorch/vision torchvision
ENV BUILD_VERSION=0.17.0
RUN cd torchvision && python3 setup.py install

# Install grid-map humble extension
# https://github.com/ethz-asl/grid_map
RUN apt-get update && \
    apt-get -y install ros-humble-grid-map \
    && rm -rf /var/lib/apt/lists/

# Install tkinter to avoid matplotlib graphs gui problems
RUN pip3 install tk

# Fixing error of /var/log/uvcdynctrl-udev.log filling SSD disk
RUN apt -y remove uvcdynctrl
RUN apt -y autoremove

# Removes colcon build setup.py warning
RUN pip3 install setuptools==58.2.0 

# Set the non-root user as the default user
USER $USERNAME

# Set ros environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
