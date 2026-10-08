#!/usr/bin/env bash
# ==============================================================================
# Automated ROS 2 Humble & Dependencies Setup Script for Ubuntu 22.04 (WSL2/Native)
# ==============================================================================
set -e

echo "=================================================================="
echo " Setting up Autonomous UGV GPS-Denied Navigation Stack on Ubuntu"
echo "=================================================================="

# 1. Update and install prerequisite tools
sudo apt update && sudo apt install -y curl gnupg lsb-release build-essential python3-pip git

# 2. Add ROS 2 apt repository if not already present
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "[*] Adding ROS 2 apt repository..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi

# 3. Install ROS 2 Humble Desktop and required autonomy packages
echo "[*] Installing ROS 2 packages, Gazebo, RTAB-Map, Nav2, and EKF..."
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-localization \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-slam \
    ros-humble-rtabmap-odom \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    python3-colcon-common-extensions

# 4. Source ROS 2 setup in bashrc if needed
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash

# 5. Build the workspace
echo "[*] Building workspace with colcon..."
colcon build --symlink-install

echo "=================================================================="
echo "[+] Workspace built successfully!"
echo "    To launch the full system, run:"
echo "    source install/setup.bash"
echo "    ros2 launch autonomous_ugv_bringup full_system.launch.py"
echo "=================================================================="
