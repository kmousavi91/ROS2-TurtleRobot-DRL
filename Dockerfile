# Start from the base ROS 2 Foxy image
FROM ros:foxy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO foxy

# ----------------------------------------------------
# FIXED STEP 2: Dependency Installation (ROS, Gazebo, PIP, and GUI tools)
# Added 'x11-utils' for GUI forwarding (Gazebo) and 'python3-pip'.
# ----------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        # GUI support (needed for Gazebo to be "showed")
        x11-utils \
        # General ROS 2 build tools and simulator bridge
        python3-colcon-common-extensions \
        ros-$ROS_DISTRO-gazebo-ros-pkgs \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-xacro \
        ros-$ROS_DISTRO-launch-ros \
        # Package runtime dependencies
        ros-$ROS_DISTRO-geometry-msgs \
        ros-$ROS_DISTRO-nav-msgs \
        ros-$ROS_DISTRO-sensor-msgs \
        ros-$ROS_DISTRO-std-msgs \
        # System packages
        python3-numpy \
        gazebo11 \
        # Install Python PIP for ML dependencies
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------
# NEW STEP: Install Machine Learning Dependencies (TensorFlow)
# This installs the CPU version of TensorFlow.
# ----------------------------------------------------
RUN pip3 install tensorflow

# Create and set up the workspace directory
WORKDIR /ros2_ws

# ----------------------------------------------------
# COPY Source Code
# ----------------------------------------------------
COPY src/turtlebot_nav /ros2_ws/src/turtlebot_nav

# Build the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-select turtlebot_nav --symlink-install

# REMOVED ENTRYPOINT: We handle environment sourcing in the 'docker run' command now.

# Default command to start a shell
CMD ["/bin/bash"]

