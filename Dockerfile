# Use the official ROS 2 Humble base image
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    sudo \
    wget \
    curl \
    python3 \
    python3-pip \
    pkg-config \
    libx11-dev \
    libgl1-mesa-dev \
    libglew-dev \
    wayland-protocols \
    libc++-dev \
    libepoxy-dev \
    ninja-build \
    libavdevice-dev \
    libeigen3-dev \
    python3-colcon-common-extensions \ 
    build-essential \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

    WORKDIR /colcon_ws
    COPY src src


    WORKDIR /colcon_ws/src

    # Clone Pangolin repository
    RUN git clone https://github.com/stevenlovegrove/Pangolin
    
    # Change to Pangolin directory
    WORKDIR /colcon_ws/src/Pangolin
    

    RUN chmod +x ./scripts/install_prerequisites.sh
    # Check recommended dependencies
    RUN ./scripts/install_prerequisites.sh --dry-run recommended || true
    
    # Install recommended dependencies
    RUN ./scripts/install_prerequisites.sh recommended
    
    # Configure and build Pangolin
    RUN cmake -B build
    RUN cmake --build build -j4
    
    # Install Pangolin
    RUN sudo cmake --install build

WORKDIR /colcon_ws
#COPY src src
SHELL ["/bin/bash", "-c"]


RUN source /opt/ros/humble/setup.bash && colcon build 
# Build the workspace


# Source the ROS 2 setup file
#RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc

# Set the entry point to a ROS 2 command or script
#ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp && ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=RealSense_D435i -p image_seq:=weed"]
CMD ["bash"]