#!/usr/bin/env bash
# Ubuntu 24.04(WSL 포함)에 ROS 2 Jazzy desktop + rosbag2 설치. 공식 절차 그대로.
#   bash tools/setup_ros_jazzy.sh
set -euo pipefail

sudo apt-get update
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe

# ros-apt-source 패키지로 저장소·키 등록 (docs.ros.org Jazzy 설치 문서 방식)
VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-rosbag2-storage-mcap python3-numpy

grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

# MediaPipe는 apt에 없어서 venv에 설치. --system-site-packages라야 rclpy가 보인다.
REPO=$(cd "$(dirname "$0")/.." && pwd)
sudo apt-get install -y python3-venv
python3 -m venv --system-site-packages "$REPO/.venv-ros"
"$REPO/.venv-ros/bin/pip" install -r "$REPO/requirements.txt"
mkdir -p "$REPO/models"
[ -f "$REPO/models/hand_landmarker.task" ] || curl -L -o "$REPO/models/hand_landmarker.task" \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task

source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
"$REPO/.venv-ros/bin/python" -c "import rclpy, mediapipe; print('rclpy + mediapipe OK')"
ros2 pkg list | grep -c . | xargs echo "packages:"
