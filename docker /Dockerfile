# Set the ROS distribution as an argument, defaulting to 'jazzy'
ARG ROS_DISTRO=jazzy

# The base image comes from the official ROS repository hosted on Docker Hub
FROM ros:${ROS_DISTRO}-ros-base

# Set the maintainer information for this Dockerfile
LABEL maintainer="AutomaticAddison<automatic_addison@todo.com>"

# Set environment variables
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive

# Set the default shell to bash for RUN commands
SHELL ["/bin/bash", "-c"]

# Update the system and install essential tools
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends apt-utils wget curl git build-essential \
    vim sudo lsb-release locales bash-completion tzdata gosu gedit htop nano libserial-dev

# Install additional tools required for ROS 2 development
RUN apt-get update -q && \
    apt-get install -y gnupg2 iputils-ping usbutils \
    python3-argcomplete python3-colcon-common-extensions python3-networkx python3-pip python3-rosdep python3-vcstool

# Set up the ROS 2 environment
RUN rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /root/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Install additional ROS 2 packages
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-reconfigure

# Install Mesa graphics drivers
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:kisak/kisak-mesa

# Create necessary directories
RUN mkdir -p /etc/udev/rules.d && \
    mkdir -p /root/ros2_ws/src/XP-Robotics-Assessment

# Copy the entire project into the container
COPY . /root/ros2_ws/src/XP-Robotics-Assessment

# Copy configuration files from docker directory
COPY docker/workspace.sh /root/
COPY docker/entrypoint.sh /root/
COPY docker/bash_aliases.txt /root/.bashrc_aliases

# Make scripts executable
RUN chmod +x /root/workspace.sh /root/entrypoint.sh

# Add custom bash aliases
RUN cat /root/.bashrc_aliases >> /root/.bashrc

# Run the workspace setup script
WORKDIR /root
RUN ./workspace.sh

# Ensure the ROS 2 workspace is sourced in every new shell
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint for the container
ENTRYPOINT ["/root/entrypoint.sh"]

# Keep the container running
CMD ["/bin/bash", "-c", "tail -f /dev/null"]
