FROM osrf/ros:humble-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER root

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME
RUN sudo usermod --append --groups dialout $USERNAME

# Update all packages
# RUN sudo apt update && sudo apt upgrade -y
# Install basic tools
RUN apt-get update && \
    apt-get install -y sudo git wget curl gnupg2 lsb-release mesa-utils python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Python + system deps
RUN apt-get update && \
    apt-get install -y v4l-utils python3-opencv && \
    rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python deps
RUN python3 -m pip install --upgrade pip ipython ipykernel ultralytics scikit-learn ikpy

# Install ROS Humble packages (only what you need)
RUN apt-get update && \
    apt-get install -y \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-robot-state-publisher \
        ros-humble-twist-stamper \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-gz-ros2-control \
        '~nros-humble-rqt*' \
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-twist-mux \
        ros-humble-rplidar-ros \
        ros-humble-joy \
        libserial-dev \
        psmisc && \
    rm -rf /var/lib/apt/lists/*

# Rosdep update
RUN rosdep update

USER $USERNAME

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################