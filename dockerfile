# Use the official ROS 2 Humble base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV QT_QPA_PLATFORM=xcb

# Install dependencies for GUI applications
RUN apt-get update && apt-get install -y \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    libqt5x11extras5 \
    python3-colcon-common-extensions \
    build-essential \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Create the ROS 2 workspace
WORKDIR /bhf_assessment1

# Copy the package source code to the workspace
COPY src src

# Change the default shell to Bash
SHELL ["/bin/bash", "-c"]

# Source the ROS environment and build the workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

# Source the workspace setup script
RUN echo "source install/setup.bash " >> /root/.bashrc

#ENTRYPOINT [" bhf_assessment1/entrypoint.sh "]

ENTRYPOINT [ "/bin/bash","-c","source install/setup.bash && ros2 run bhf_vo_new bhf_vo_new src/"]


# Default command when starting the container
#CMD ["bash"]
