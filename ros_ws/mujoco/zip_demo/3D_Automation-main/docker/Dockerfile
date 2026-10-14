FROM osrf/ros:humble-desktop-full-jammy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    git \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    python3-vcstool \
    libegl1 \
    libgl1 \
    libglew2.2 \
    libglfw3 \
    libosmesa6 \
    patchelf \
    ros-humble-cv-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-image-transport \
    ros-humble-gripper-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    "numpy<2.0" \
    "mujoco==3.3.7"

ENV MUJOCO_GL=glfw

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo '[ -f /workspace/3D_Automation/install/setup.bash ] && source /workspace/3D_Automation/install/setup.bash' >> /root/.bashrc