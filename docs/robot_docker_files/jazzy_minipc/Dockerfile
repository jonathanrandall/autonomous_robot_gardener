FROM osrf/ros:jazzy-desktop-full

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

# # Python + system deps
# RUN apt-get update && \
#     apt-get install -y v4l-utils python3-opencv && \
#     rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python deps
# RUN python3 -m pip install --upgrade pip ipython ipykernel ultralytics scikit-learn ikpy

# Install ROS jazzy packages (only what you need)
RUN apt-get update && \
    apt-get install -y \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-twist-stamper \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-gz-ros2-control \
        '~nros-jazzy-rqt*' \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-twist-mux \
        ros-jazzy-rplidar-ros \
        ros-jazzy-joy \
        libserial-dev \
        psmisc && \
        ros-jazzy-slam-toolbox && \
        ros-jazzy-nav2-bringup && \
        ros-jazzy-navigation2 && \
    rm -rf /var/lib/apt/lists/*

# Rosdep update
RUN rosdep update

RUN apt-get update && \
    apt-get install -y \
    git wget curl gnupg2 lsb-release mesa-utils \
    python3-pip python3-venv python3-dev build-essential \
    v4l-utils python3-opencv libgl1 libserial-dev psmisc \
    && rm -rf /var/lib/apt/lists/*


RUN python3 -m venv /opt/venv && \ 
    /opt/venv/bin/pip install --upgrade pip setuptools wheel && \ 
    /opt/venv/bin/pip install ipython ipykernel ultralytics scikit-learn




USER $USERNAME

# pip install pybullet


# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# sudo groupadd -g 110 render
# sudo usermod -aG render ubuntu