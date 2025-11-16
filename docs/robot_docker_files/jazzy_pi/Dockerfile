# Use ARM64 Ubuntu 24.04 (Jazzy compatible)
FROM arm64v8/ubuntu:24.04

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive

# sudo chown -R ros:ros /robot_gardener/ros_pi_ws
# Default non-root user
ARG USERNAME=ros
ARG USER_UID=2000
ARG USER_GID=2000

RUN groupadd -g $USER_GID $USERNAME && \
    useradd -m -u $USER_UID -g $USER_GID -s /bin/bash $USERNAME && \
    groupadd -f video && \
    groupadd -f i2c && \
    usermod -aG video,i2c,sudo $USERNAME && \
    mkdir -p /etc/sudoers.d && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# ----------------------------
# Basic tools & ROS 2 repo setup
# ----------------------------
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release sudo build-essential \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 Jazzy repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list

# ----------------------------
# Install ROS 2 Jazzy packages
# ----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-perception \
    ros-jazzy-demo-nodes-py \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# ----------------------------
# Create non-root user & groups
# ----------------------------

# ----------------------------
# Install general tools, Python & OpenCV/Arducam deps
# ----------------------------
RUN apt-get update && apt-get install -y \
    git wget curl python3-pip python3-venv python3-dev \
    python3-opencv libopencv-dev libgl1 mesa-utils \
    v4l-utils i2c-tools libi2c-dev psmisc \
    python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------
# Python virtual environment for YOLO & Arducam
# ----------------------------
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip setuptools wheel && \
    /opt/venv/bin/pip install ipython ipykernel scikit-learn \
    ArducamDepthCamera opencv-python "numpy<2.0.0"

# ----------------------------
# Switch to non-root
# ----------------------------

# RUN chown -R ros:ros /robot_gardener
USER $USERNAME
WORKDIR /home/$USERNAME

# pip install adafruit-circuitpython-motor
# pip install adafruit-circuitpython-pca9685
# pip install RPi.GPIO
# sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
# pip install PyYAML
# sudo chown -R ros:ros /robot_gardener/ros_pi_ws


# Source ROS and alias venv activation
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "alias env_activate='source /opt/venv/bin/activate'" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]
