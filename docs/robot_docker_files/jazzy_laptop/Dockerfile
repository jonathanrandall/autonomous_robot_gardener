FROM osrf/ros:humble-desktop
#sudo apt update && sudo apt install -y mesa-utils
#sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
#sudo apt install ros-humble-urdf-sim-tutorial
#sudo apt install '~nros-humble-rqt*'
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



# Install Git
RUN sudo apt update && sudo apt install -y git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    mesa-utils

RUN sudo apt-get install -y python3-pip
# pip install "numpy==1.24.3" for inverse kinematics
# sudo apt install --reinstall python3-numpy python3-scipy python3-matplotlib
# System + Python dependencies
RUN sudo apt-get install -y \
    v4l-utils \
    python3-opencv

RUN sudo usermod -aG video $USERNAME
RUN sudo usermod -aG dialout $USERNAME
RUN sudo usermod -aG audio $USERNAME
# Set environment variables for ROS
# Upgrade pip and install Python packages
RUN python3 -m pip install --upgrade pip ipython ipykernel ultralytics scikit-learn ikpy 
#ipywidgets

# Add user to video group for webcam access




# RUN sudo rm /etc/apt/sources.list.d/ros2.list
# RUN sudo rm /etc/apt/sources.list.d/ros2-latest.list


# Download the new official ROS key
# RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
#   -o /usr/share/keyrings/ros-archive-keyring.gpg

# # # Add the new repo for ROS 2 Humble (Jammy)
# RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
#     http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
# Download the new official ROS key
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
#   -o /usr/share/keyrings/ros-archive-keyring.gpg

# # # Add the new repo for ROS 2 Humble (Jammy)
# RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
#   http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

  # Set up ROS 2 Humble APT repo from scratch
# RUN apt update && apt install -y curl gnupg lsb-release

# # Clean old keys/repos
# RUN rm -f /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2.list

# # Install new ROS key and source
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
#   -o /usr/share/keyrings/ros-archive-keyring.gpg

# RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
#   http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
#   > /etc/apt/sources.list.d/ros2.list

  # Essential tools and setup
# Ensure tools for setting up repositories are available
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Add ROS 2 GPG key and source list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Update apt again now that the correct key/repo is in place
RUN apt-get update
# RUN apt-get update && apt-get install -y curl gnupg lsb-release && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
#     echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
#     apt-get update


# Install ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-twist-stamper \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gz-ros2-control
    # ros-humble-gz-sim 
    # ros-humble-gz-ros2-control-system \
    # ros-humble-ros2-controllers \
    # ros-humble-twist-mux \
    # ros-humble-rplidar-ros

RUN sudo apt-get update
RUN sudo apt install -y '~nros-humble-rqt*'
RUN sudo apt install -y  \ 
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros2-controllers \
    ros-humble-twist-mux \
    ros-humble-rplidar-ros \
    ros-humble-joy

# RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
#     sh -c 'echo "deb [arch=amd64] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'



# RUN sudo apt-get install -y \
#     ignition-fortress \
#     ros-humble-ros-ign \
#     ros-humble-ros-ign-bridge \
#     ros-humble-ros-gz-sim
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt install -y libserial-dev
RUN sudo apt install -y psmisc
# RUN usermod -aG dialout $(USERNAME)

# RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# RUN sudo apt-get update
# RUN sudo apt-get install -y ignition-fortress




# RUN sudo apt install -y ros-humble-urdf-sim-tutorial ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros

# RUN sudo apt-get install -y \ 
#     libgz-cmake3-dev \
#     libgz-plugin2-dev \
#     libgz-common5-dev
#     libgz-sim7-dev 
#     gz-tools2 \
#     libgz-tools2-dev \
#     libgz-plugin2-dev \
#     libgz-common5-dev 
# Rosdep update
RUN rosdep update

USER $USERNAME

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################