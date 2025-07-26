FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget

# Add Gazebo (Fortress) key and repository
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y ignition-fortress

# Install Gazebo Fortress + ROS 2 related tools
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros-ign \
    ros-humble-xacro \
    ros-humble-rviz2 \
    python3-pip && \
    pip3 install matplotlib && \
    rm -rf /var/lib/apt/lists/*

# Install additional Python dependencies
RUN pip3 install \
    numpy \
    scipy \
    ikpy 

# Set up environment
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DOMAIN_ID=0 \
    ROS_VERSION=2 \
    ROS_DISTRO=humble

WORKDIR /workspace

# Source setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
